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

#include <nuttx/net/net.h>
#include <nuttx/net/tcp.h>

#include "tcp/tcp.h"

#ifdef CONFIG_NET_TCP_KEEPALIVE

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_getsockopt
 *
 * Description:
 *   tcp_getsockopt() retrieves the value for the option specified by the
 *   'option' argument for the socket specified by the 'psock' argument.  If
 *   the size of the option value is greater than 'value_len', the value
 *   stored in the object pointed to by the 'value' argument will be silently
 *   truncated. Otherwise, the length pointed to by the 'value_len' argument
 *   will be modified to indicate the actual length of the 'value'.
 *
 *   The 'level' argument specifies the protocol level of the option. To
 *   retrieve options at the socket level, specify the level argument as
 *   SOL_SOCKET; to retrieve options at the TCP-protocol level, the level
 *   argument is SOL_CP.
 *
 *   See <sys/socket.h> a complete list of values for the socket-level
 *   'option' argument.  Protocol-specific options are are protocol specific
 *   header files (such as netinet/tcp.h for the case of the TCP protocol).
 *
 * Parameters:
 *   psock     Socket structure of the socket to query
 *   level     Protocol level to set the option
 *   option    identifies the option to get
 *   value     Points to the argument value
 *   value_len The length of the argument value
 *
 * Returned Value:
 *   Returns zero (OK) on success.  On failure, it returns a negated errno
 *   value to indicate the nature of the error.  See psock_getsockopt() for
 *   the complete list of appropriate return error codes.
 *
 ****************************************************************************/

int tcp_getsockopt(FAR struct socket *psock, int option,
                   FAR void *value, FAR socklen_t *value_len)
{
  FAR struct tcp_conn_s *conn;
  int ret;

  DEBUGASSERT(psock != NULL && value != NULL && value_len != NULL &&
              psock->s_conn != NULL);
  conn = (FAR struct tcp_conn_s *)psock->s_conn;

  /* REVISIT: Should we check if SO_KEEPALIVE is enabled?  Or just blindly
   * return the settings?  Lets do the latter.
   */

  switch (option)
    {
      case TCP_NODELAY: /* Avoid coalescing of small segments. */
        ret = -ENOSYS;
        break;

      case TCP_KEEPIDLE:  /* Start keeplives after this IDLE period */
        {
          FAR uint32_t *keepidle = (FAR uint32_t *)value;
          *keepidle  = conn->keepidle;
          *value_len = sizeof(uint32_t);
          ret = OK;
        }
        break;

      case TCP_KEEPINTVL: /* Interval between keepalives */
        {
          FAR uint16_t *keepintvl = (FAR uint16_t *)value;
          *keepintvl = conn->keepintvl;
          *value_len = sizeof(uint16_t);
          ret = OK;
        }
        break;

      case TCP_KEEPCNT:   /* Number of keepalives before death */
        {
          FAR uint8_t *keepcnt = (FAR uint8_t *)value;
          *keepcnt   = conn->keepcnt;
          *value_len = sizeof(uint8_t);
          ret = OK;
        }
        break;

      default:
        ret = -EINVAL;
        break;
    }

  return ret;
}

#endif /* CONFIG_NET_TCP_KEEPALIVE */
