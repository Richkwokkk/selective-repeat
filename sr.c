#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "emulator.h"
#include "sr.h"

/* ******************************************************************
   Go Back N protocol.  Adapted from J.F.Kurose
   ALTERNATING BIT AND GO-BACK-N NETWORK EMULATOR: VERSION 1.2  

   Network properties:
   - one way network delay averages five time units (longer if there
   are other messages in the channel for GBN), but can be larger
   - packets can be corrupted (either the header or the data portion)
   or lost, according to user-defined probabilities
   - packets will be delivered in the order in which they were sent
   (although some can be lost).

   Modifications: 
   - removed bidirectional GBN code and other code not used by prac. 
   - fixed C style to adhere to current programming style
   - added GBN implementation
**********************************************************************/

#define RTT  16.0       /* round trip time.  MUST BE SET TO 16.0 when submitting assignment */
#define WINDOWSIZE 6    /* the maximum number of buffered unacked packet */
#define SEQSPACE 12      /* the min sequence space for GBN must be at least windowsize * 2 */
#define NOTINUSE (-1)   /* used to fill header fields that are not being used */

/* generic procedure to compute the checksum of a packet.  Used by both sender and receiver  
   the simulator will overwrite part of your packet with 'z's.  It will not overwrite your 
   original checksum.  This procedure must generate a different checksum to the original if
   the packet is corrupted.
*/
int ComputeChecksum(struct pkt packet)
{
  int checksum = 0;
  int i;

  checksum = packet.seqnum;
  checksum += packet.acknum;
  for ( i=0; i<20; i++ ) 
    checksum += (int)(packet.payload[i]);

  return checksum;
}

bool IsCorrupted(struct pkt packet)
{
  if (packet.checksum == ComputeChecksum(packet))
    return (false);
  else
    return (true);
}


/********* Sender (A) variables and functions ************/

static struct pkt buffer[WINDOWSIZE];  /* array for storing packets waiting for ACK */
static int windowcount;                /* the number of packets currently awaiting an ACK */
static int A_nextseqnum;               /* the next sequence number to be used by the sender */
static int send_base;                  /* the base of the send window */
static bool acked[WINDOWSIZE];         /* tracks which packets are ACKed */ 
static int timer_ids[WINDOWSIZE];      /* timer ID for each packet */

/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message)
{
  struct pkt sendpkt;
  int i;
  int buf_index;

  /* if not blocked waiting on ACK */
  if ( windowcount < WINDOWSIZE) {
    if (TRACE > 1)
      printf("----A: New message arrives, send window is not full, send new messge to layer3!\n");

    /* create packet */
    sendpkt.seqnum = A_nextseqnum;
    sendpkt.acknum = NOTINUSE;
    for ( i=0; i<20 ; i++ ) 
      sendpkt.payload[i] = message.data[i];
    sendpkt.checksum = ComputeChecksum(sendpkt); 

    /* put packet in window buffer */
    buf_index = A_nextseqnum % WINDOWSIZE;
    buffer[buf_index] = sendpkt;
    acked[buf_index] = false;

    /* send out packet */
    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3 (A, sendpkt);

    /* In SR, start a timer for this specific packet */
    timer_ids[buf_index] = sendpkt.seqnum;
    starttimer(A, RTT);

    /* get next sequence number, wrap back to 0 */
    windowcount++;
    A_nextseqnum = (A_nextseqnum + 1) % SEQSPACE;  
  }
  /* if blocked,  window is full */
  else {
    if (TRACE > 0)
      printf("----A: New message arrives, send window is full\n");
    window_full++;
  }
}


/* called from layer 3, when a packet arrives for layer 4 
   In this practical this will always be an ACK as B never sends data.
*/
void A_input(struct pkt packet)
{
  int i;

  /* if received ACK is not corrupted */ 
  if (!IsCorrupted(packet)) {
    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n",packet.acknum);
    total_ACKs_received++;

    /* check if ACK is within current window */
    if (((send_base <= (send_base + WINDOWSIZE - 1) % SEQSPACE) && 
         (packet.acknum >= send_base && packet.acknum <= (send_base + WINDOWSIZE - 1) % SEQSPACE)) || 
        ((send_base > (send_base + WINDOWSIZE - 1) % SEQSPACE) && 
         (packet.acknum >= send_base || packet.acknum <= (send_base + WINDOWSIZE - 1) % SEQSPACE))) {
      
      /* calculate buffer index */
      i = packet.acknum % WINDOWSIZE;

        if (acked[i]) {
          if (TRACE > 0)
          printf("----A: duplicate ACK received, do nothing!\n");
      } else {
        if (TRACE > 0)
          printf("----A: ACK %d is not a duplicate\n", packet.acknum);
        new_ACKs++;
        
        acked[i] = true;
        /* stop timer for this packet */
        stoptimer(A);

        /* if the ACK is for the first packet in the window, 
        we need to move the window forward */
        if (packet.acknum == send_base) {
          while (acked[send_base % WINDOWSIZE]) {
            acked[send_base % WINDOWSIZE] = false;
            send_base = (send_base + 1) % SEQSPACE;
            windowcount--;

            if (windowcount == 0) {
              break;
            }
          }
        }

        /* restart timer for the new window */
        if (windowcount > 0) {
          for (i = 0; i < windowcount; i++) {
            int seqnum = (send_base + i) % WINDOWSIZE;
            if (seqnum == A_nextseqnum) 
              break;
            i = seqnum % WINDOWSIZE;
            if (!acked[i]) {
              starttimer(A, RTT);
              break;
            }
          }
        }   
      } 
    } else {
      if (TRACE > 0)
        printf ("----A: ACK outside window, do nothing!\n");
    }
  } else {
    if (TRACE > 0)
        printf ("----A: corrupted ACK is received, do nothing!\n");
  }
}


/* called when A's timer goes off */
void A_timerinterrupt(void)
{
  int i;

  if (TRACE > 0)
    printf("----A: time out,resend packets!\n");

  /* In SR, find the earliest unACKed packet and resend just that one */
  for (i = 0; i < windowcount; i++) {
    i = (send_base + i) % SEQSPACE % WINDOWSIZE;
    if (!acked[i] && (send_base + i) % SEQSPACE != A_nextseqnum) {
      if (TRACE > 0)
        printf ("---A: resending packet %d\n", (buffer[i]).seqnum);
      
      tolayer3(A, buffer[i]);
      packets_resent++;
      starttimer(A, RTT);
      break;
    }
  }
}

/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void)
{
  int i;

  /* initialise A's window, buffer and sequence number */
  A_nextseqnum = 0;  /* A starts with seq num 0, do not change this */
  send_base = 0;
  windowcount = 0;

  /* Initialize acked array */
  for (i = 0; i < WINDOWSIZE; i++) {
    acked[i] = true;
    timer_ids[i] = NOTINUSE;
  }
}


/********* Receiver (B)  variables and procedures ************/

static int B_nextseqnum;   /* the sequence number for the next packets sent by B */
static struct pkt rcv_buffer[WINDOWSIZE]; /* buffer for out-of-order packets */
static bool rcv_acked[WINDOWSIZE];        /* tracks which packets are in the buffer */
static int rcv_base;                      /* base of the receive window */

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
  struct pkt sendpkt;
  int i;
  int packet_index;

  /* if packet is not corrupted */
  if (!IsCorrupted(packet)) {
    int rcv_last = (rcv_base + WINDOWSIZE - 1) % SEQSPACE;

    /* Check if the seqnum is within our receive window */ 
    bool in_window = false;
    if ((rcv_base <= rcv_last && packet.seqnum >= rcv_base && packet.seqnum <= rcv_last) ||
        (rcv_base > rcv_last && (packet.seqnum >= rcv_base || packet.seqnum <= rcv_last))) {
      in_window = true;
    }

    if (in_window) {
      /* SR: Accept packet in window and send ACK for it */
      if (TRACE > 0)
        printf("----B: packet %d is correctly received, send ACK!\n", packet.seqnum);
      packets_received++;

      /* Buffer the packet if not already received */
      packet_index = packet.seqnum % WINDOWSIZE;
      rcv_buffer[packet_index] = packet;
      rcv_acked[packet_index] = true;

      if (packet.seqnum == rcv_base) {
        while (rcv_acked[rcv_base % WINDOWSIZE]) {
          /* Deliver in-order packets to layer 5 */
          tolayer5(B, rcv_buffer[rcv_base % WINDOWSIZE].payload);
          rcv_acked[rcv_base % WINDOWSIZE] = false;
          rcv_base = (rcv_base + 1) % SEQSPACE;
        }
      }

      /* send an ACK for the received packet */
      sendpkt.acknum = packet.seqnum;     
    } else {
      /* Packet is outside our window - could be a duplicate */
      if (TRACE > 0) 
        printf("----B: packet corrupted or not expected sequence number, resend ACK!\n");

      /* For SR, still ACK this packet (even if it's before our window) */
      sendpkt.acknum = packet.seqnum;
    }
  } else {
    /* Packet is corrupted */
    if (TRACE > 0)
      printf("----B: packet corrupted or not expected sequence number, resend ACK!\n");

    /* No valid ACK to send */
    if (rcv_base == 0)
      sendpkt.acknum = SEQSPACE - 1;
    else
      sendpkt.acknum = rcv_base - 1;
  }

  /* create packet */
  sendpkt.seqnum = B_nextseqnum;
  B_nextseqnum = (B_nextseqnum + 1) % 2;

  /* we don't have any data to send.  fill payload with 0's */
  for (i = 0; i < 20; i++) {
    sendpkt.payload[i] = '0';

  /* computer checksum */
  sendpkt.checksum = ComputeChecksum(sendpkt);

  /* send out packet */
  tolayer3(B, sendpkt);
  }
}

/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void)
{
  int i;

  B_nextseqnum = 0;
  rcv_base = 0;

  /* Initialize receiver buffer */
  for (i = 0; i < WINDOWSIZE; i++) {
    rcv_acked[i] = false;
  }
}

/******************************************************************************
 * The following functions need be completed only for bi-directional messages *
 *****************************************************************************/

/* Note that with simplex transfer from a-to-B, there is no B_output() */
void B_output(struct msg message)  
{
}

/* called when B's timer goes off */
void B_timerinterrupt(void)
{
}

