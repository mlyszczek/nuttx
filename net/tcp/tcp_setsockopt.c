/****************************************************************************
 * net/tcp/tcp_setsockopt.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <netinet/tcp.h>

#include <nuttx/clock.h>
#include <nuttx/net/net.h>
#include <nuttx/net/tcp.h>

#include "socket/socket.h"
#include "tcp/tcp.h"

#ifdef CONFIG_NET_TCP_KEEPALIVE

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_setsockopt
 *
 * Description:
 *   tcp_setsockopt() sets the TCP-protocol option specified by the
 *   'option' argument to the value pointed to by the 'value' argument for
 *   the socket specified by the 'psock' argument.
 *
 *   See <netinet/tcp.h> for the a complete list of values of TCP protocol
 *   options.
 *
 * Input Parameters:
 *   psock     Socket structure of socket to operate on
 *   option    identifies the option to set
 *   value     Points to the argument value
 *   value_len The length of the argument value
 *
 * Returned Value:
 *   Returns zero (OK) on success.  On failure, it returns a negated errno
 *   value to indicate the nature of the error.  See psock_setcockopt() for
 *   the list of possible error values.
 *
 ****************************************************************************/

int tcp_setsockopt(FAR struct socket *psock, int option,
                   FAR const void *value, socklen_t value_len)
{
  FAR struct tcp_conn_s *conn;
  int ret;

  DEBUGASSERT(psock != NULL && value != NULL && psock->s_conn != NULL);
  conn = (FAR struct tcp_conn_s *)psock->s_conn;

  /* All of the TCP protocol options apply only for connected, TCP sockets */

  if (psock->s_type != SOCK_STREAM || !_SS_ISCONNECTED(psock->s_flags))
    {
      nerr("ERROR: Not connected\n");
      return -ENOTCONN;
    }

  /* It makes no sense to set or query these options if SO_KEEPALIVE is not
   * enabled?
   */

  if (!_SO_GETOPT(psock->s_options, SO_KEEPALIVE))
    {
      nerr("ERROR: Keep alive not enabled\n");
      return -EINVAL;
    }

  /* Handle the Keep-Alive option */

  switch (option)
    {
      case TCP_NODELAY: /* Avoid coalescing of small segments. */
        nerr("ERROR: TCP_NODELAY not supported\n");
        ret = -ENOSYS;
        break;

      case TCP_KEEPIDLE:  /* Start keeplives after this IDLE period */
        if (value_len != sizeof(int))
          {
            ret = -EDOM;
          }
        else
          {
            int keepidle = *(FAR int *)value;

            if (keepidle < 0)
              {
                nerr("ERROR: TCP_KEEPIDLE value out of range: %d\n", keepidle);
                ret = -EDOM;
              }
            else
              {
                conn->keepidle = (uint32_t)keepidle;
                conn->keeplive = clock_systimer();   /* Reset start time */
                ret = OK;
              }
          }
        break;

      case TCP_KEEPINTVL: /* Interval between keepalives */
        if (value_len != sizeof(int))
          {
            ret = -EDOM;
          }
        else
          {
            int keepintvl = *(FAR int *)value;

            if (keepintvl < 0 || keepintvl > UINT16_MAX)
              {
                nerr("ERROR: TCP_KEEPINTVL value out of range: %d\n", keepintvl);
                ret = -EDOM;
              }
            else
              {
                conn->keepintvl = (uint16_t)keepintvl;
                conn->keeplive = clock_systimer();   /* Reset start time */
                ret = OK;
              }
          }
        break;

      case TCP_KEEPCNT:   /* Number of keepalives before death */
        if (value_len != sizeof(int))
          {
            ret = -EDOM;
          }
        else
          {
            int keepcnt = *(FAR int *)value;

            if (keepcnt < 0 || keepcnt > UINT8_MAX)
              {
                nerr("ERROR: TCP_KEEPCNT value out of range: %d\n", keepcnt);
                return -EDOM;
              }
            else
              {
                conn->keepcnt = (uint8_t)keepcnt;
                conn->keeplive = clock_systimer();   /* Reset start time */
                ret = OK;
              }
          }
        break;

      default:
        nerr("ERROR: Unrecognized TCP option: %d\n", option);
        ret = -ENOPROTOOPT;
        break;
    }

  return ret;
}

#endif /* CONFIG_NET_TCP_KEEPALIVE */
