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

#define RTT 16.0      /* round trip time.  MUST BE SET TO 16.0 when submitting assignment */
#define WINDOWSIZE 6  /* the maximum number of buffered unacked packet */
#define SEQSPACE 7    /* the min sequence space for GBN must be at least windowsize + 1 */
#define NOTINUSE (-1) /* used to fill header fields that are not being used */

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
  for (i = 0; i < 20; i++)
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

/* State variables for sender */
static struct pkt buffer[WINDOWSIZE]; /* array for storing packets waiting for ACK */
static bool acked[WINDOWSIZE];        /* indicates whether packet has been ACKed */
static int windowbase;                /* base sequence number of the window */
static int A_nextseqnum;              /* the next sequence number to be used by the sender */
static int windowcount;               /* the number of packets currently awaiting an ACK */
static int oldest_unacked;            /* sequence number of the oldest unacked packet */
static bool in_window[SEQSPACE];      /* tracks if a sequence number is in the current window */

/* Find the oldest unacknowledged packet to time */
static void find_oldest_unacked(void)
{
  int i;
  int seq;
  oldest_unacked = -1;

  /* Start from the windowbase and find the first unacked packet */
  for (i = 0; i < windowcount; i++)
  {
    seq = (windowbase + i) % SEQSPACE;
    if (!acked[seq % WINDOWSIZE] && in_window[seq])
    {
      oldest_unacked = seq;
      return;
    }
  }
}

/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message)
{
  struct pkt sendpkt;
  int i;
  int index;

  /* if not blocked waiting on ACK */
  if (windowcount < WINDOWSIZE)
  {
    if (TRACE > 1)
      printf("----A: New message arrives, send window is not full, send new messge to layer3!\n");

    /* create packet */
    sendpkt.seqnum = A_nextseqnum;
    sendpkt.acknum = NOTINUSE;
    for (i = 0; i < 20; i++)
      sendpkt.payload[i] = message.data[i];
    sendpkt.checksum = ComputeChecksum(sendpkt);

    /* put packet in window buffer */
    index = A_nextseqnum % WINDOWSIZE;
    buffer[index] = sendpkt;
    acked[index] = false;
    in_window[A_nextseqnum] = true;
    windowcount++;

    /* send out packet */
    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3(A, sendpkt);

    /* If this is the first unacked packet, start the timer */
    if (oldest_unacked == -1)
    {
      oldest_unacked = A_nextseqnum;
      starttimer(A, RTT);
    }

    /* get next sequence number, wrap back to 0 */
    A_nextseqnum = (A_nextseqnum + 1) % SEQSPACE;
  }
  /* if blocked,  window is full */
  else
  {
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
  int index;

  /* if received ACK is not corrupted */
  if (!IsCorrupted(packet))
  {
    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n", packet.acknum);
    total_ACKs_received++;

    /* check if ACK is within current window */
    if (((windowbase <= (windowbase + windowcount - 1) % SEQSPACE) &&
         (packet.acknum >= windowbase && packet.acknum <= (windowbase + windowcount - 1) % SEQSPACE)) ||
        ((windowbase > (windowbase + windowcount - 1) % SEQSPACE) &&
         (packet.acknum >= windowbase || packet.acknum <= (windowbase + windowcount - 1) % SEQSPACE)))
    {
      index = packet.acknum % WINDOWSIZE;

      /* Only process if not already ACKed */
      if (!acked[index] && in_window[packet.acknum])
      {
        if (TRACE > 0)
          printf("----A: ACK %d is not a duplicate\n", packet.acknum);
        new_ACKs++;

        /* mark as ACKed */
        acked[index] = true;

        /* If this was the packet we were timing, stop timer and find next to time */
        if (packet.acknum == oldest_unacked)
        {
          stoptimer(A);
          find_oldest_unacked();

          /* If there are still unacked packets, restart timer */
          if (oldest_unacked != -1)
          {
            starttimer(A, RTT);
          }
        }

        /* Slide window if base packet is ACKed */
        while (windowcount > 0 && acked[windowbase % WINDOWSIZE] && in_window[windowbase])
        {
          in_window[windowbase] = false; // Mark as no longer in window
          windowbase = (windowbase + 1) % SEQSPACE;
          windowcount--;
        }
      }
      else
      {
        if (TRACE > 0)
          printf("----A: duplicate ACK %d, do nothing!\n", packet.acknum);
      }
    }
    else
    {
      if (TRACE > 0)
        printf("----A: ACK %d outside window, do nothing!\n", packet.acknum);
    }
  }
  else
  {
    if (TRACE > 0)
      printf("----A: corrupted ACK is received, do nothing!\n");
  }
}

/* called when A's timer goes off */
void A_timerinterrupt(void)
{
  int index;

  if (oldest_unacked != -1)
  {
    index = oldest_unacked % WINDOWSIZE;

    if (TRACE > 0)
      printf("----A: time out,resend packets!\n");

    /* Only resend if still within window and not yet ACKed */
    if (!acked[index] && in_window[oldest_unacked])
    {
      /* resend just this packet */
      if (TRACE > 0)
        printf("---A: resending packet %d\n", oldest_unacked);

      tolayer3(A, buffer[index]);
      packets_resent++;

      starttimer(A, RTT);
    }
    else
    {
      /* This packet is already ACKed, find next one to time */
      find_oldest_unacked();
      if (oldest_unacked != -1)
      {
        starttimer(A, RTT);
      }
    }
  }
}

/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void)
{
  /* initialise A's window, buffer and sequence number */
  int i;
  A_nextseqnum = 0; /* A starts with seq num 0, do not change this */
  windowcount = 0;
  windowbase = 0;
  oldest_unacked = -1;

  for (i = 0; i < WINDOWSIZE; i++)
  {
    acked[i] = false;
  }

  for (i = 0; i < SEQSPACE; i++)
  {
    in_window[i] = false;
  }
}

/********* Receiver (B)  variables and procedures ************/

static int expectedseqnum;                 /* the sequence number expected next by the receiver */
static int B_nextseqnum;                   /* the sequence number for the next packets sent by B */
static struct pkt recv_buffer[WINDOWSIZE]; /* buffer for out-of-order packets */
static bool received[WINDOWSIZE];          /* indicates whether packet is received in window */
static int B_windowbase;                   /* base of the receiver window */
static bool already_received[SEQSPACE];    /* track which packets have been received already */

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
  struct pkt sendpkt;
  int i;
  int index;

  /* If not corrupted, check if in receive window */
  if (!IsCorrupted(packet))
  {
    if (TRACE > 0)
      printf("----B: packet %d is correctly received, send ACK!\n", packet.seqnum);

    /* Count ALL correctly received packets (even duplicates) */
    packets_received++;

    /* Check if packet falls within the receive window */
    if (((B_windowbase <= (B_windowbase + WINDOWSIZE - 1) % SEQSPACE) &&
         (packet.seqnum >= B_windowbase && packet.seqnum <= (B_windowbase + WINDOWSIZE - 1) % SEQSPACE)) ||
        ((B_windowbase > (B_windowbase + WINDOWSIZE - 1) % SEQSPACE) &&
         (packet.seqnum >= B_windowbase || packet.seqnum <= (B_windowbase + WINDOWSIZE - 1) % SEQSPACE)))
    {

      /* Store packet in the buffer if not already received */
      index = packet.seqnum % WINDOWSIZE;
      if (!received[index])
      {
        received[index] = true;
        recv_buffer[index] = packet;

        /* Track which packets have been delivered to app layer */
        if (!already_received[packet.seqnum])
        {
          already_received[packet.seqnum] = true;
        }

        /* If this is the base of the window, deliver it and any consecutive buffered packets */
        if (packet.seqnum == B_windowbase)
        {
          do
          {
            /* Deliver to application layer */
            tolayer5(B, recv_buffer[B_windowbase % WINDOWSIZE].payload);

            /* Mark as not received since it's been delivered */
            received[B_windowbase % WINDOWSIZE] = false;

            /* Move window base forward */
            B_windowbase = (B_windowbase + 1) % SEQSPACE;
          } while (received[B_windowbase % WINDOWSIZE]);
        }
      }

      /* Always send an ACK for the received packet */
      sendpkt.acknum = packet.seqnum;
    }
    else
    {
      /* Packet outside window - must be a duplicate from below the window */
      if (TRACE > 0)
        /*printf("----B: packet %d outside window, send ACK anyway\n", packet.seqnum);*/
        sendpkt.acknum = packet.seqnum;
    }
  }
  else
  {
    /* Packet is corrupted, do not send ACK */
    if (TRACE > 0)
      /*printf("----B: packet corrupted, do not send ACK\n");*/
      return;
  }

  /* Create ACK packet */
  sendpkt.seqnum = B_nextseqnum;
  B_nextseqnum = (B_nextseqnum + 1) % 2; /* Alternating bit for ACK seqnum */

  /* Fill payload with 0's - no data in ACKs */
  for (i = 0; i < 20; i++)
    sendpkt.payload[i] = '0';

  /* Compute checksum and send */
  sendpkt.checksum = ComputeChecksum(sendpkt);
  tolayer3(B, sendpkt);
}

/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void)
{
  int i;
  expectedseqnum = 0;
  B_nextseqnum = 1;
  B_windowbase = 0;

  for (i = 0; i < WINDOWSIZE; i++)
  {
    received[i] = false;
  }
  for (i = 0; i < SEQSPACE; i++)
  {
    already_received[i] = false;
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