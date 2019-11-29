/****************************************************************************
 * net/netlink/netlink_crypto.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/crypto/crypto_netlink.h>

#include "netlink/netlink.h"

#ifdef CONFIG_NETLINK_CRYPTO

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Helpers ******************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netlink_crypto_sendto()
 *
 * Description:
 *   Perform the sendto() operation for the NETLINK_CRYPTO protocol.
 *
 ****************************************************************************/

ssize_t netlink_crypto_sendto(FAR struct socket *psock,
                              FAR const struct nlmsghdr *nlmsg,
                              size_t len, int flags,
                              FAR const struct sockaddr_alg *to,
                              socklen_t tolen)
{
  int ret;

  DEBUGASSERT(psock != NULL && nlmsg != NULL &&
              nlmsg->nlmsg_len >= sizeof(struct nlmsghdr) &&
              len >= sizeof(struct nlmsghdr) &&
              len >= nlmsg->nlmsg_len && to != NULL &&
              tolen >= sizeof(struct sockaddr_alg));

  /* Message parsing is handled by the crypto subsystem.
   */

  ret = crypto_request_handler((NETLINK_HANDLE)psock, nlmsg, len, to);

  /* On success, return the size of the request that was processed */

  if (ret >= 0)
    {
      ret = len;
    }

  return ret;
}

/****************************************************************************
 * Name: netlink_crypto_recvfrom()
 *
 * Description:
 *   Perform the recvfrom() operation for the NETLINK_CRYPTO protocol.
 *
 ****************************************************************************/

ssize_t netlink_crypto_recvfrom(FAR struct socket *psock,
                                FAR struct nlmsghdr *nlmsg,
                                size_t len, int flags,
                                FAR struct sockaddr_alg *from)
{
  FAR struct netlink_response_s *entry;
  ssize_t ret;

  DEBUGASSERT(psock != NULL && nlmsg != NULL &&
              len >= sizeof(struct nlmsghdr));

  /* Find the response to this message.  The return value */

  entry = (FAR struct netlink_response_s *)netlink_tryget_response(psock);
  if (entry == NULL)
    {
      /* No response is variable, but presumably, one is expected.  Check
       * if the socket has been configured for non-blocking operation.
       * REVISIT:  I think there needs to be some higher level logic to
       * select Netlink non-blocking sockets.
       */

      if (_SS_ISNONBLOCK(psock->s_flags))
        {
          return -EAGAIN;
        }

      /* Wait for the response.  This should always succeed. */

      entry = (FAR struct netlink_response_s *)netlink_get_response(psock);
      DEBUGASSERT(entry != NULL);
      if (entry == NULL)
        {
          return -EPIPE;
        }
    }

  if (len < entry->msg.nlmsg_len)
    {
      kmm_free(entry);
      return -EMSGSIZE;
    }

  /* Handle the response according to the message type */

  switch (entry->msg.nlmsg_type)
    {
#warning Missing logic
      case ???:
        {
          /* Return address.  REVISIT... this is just a guess. */

          if (from != NULL)
            {
#warning Missing logic
            }

          /* The return value is the payload size, i.e., the  */

          ret = entry->msg.nlmsg_len;
        }
        break;

      default:
        nerr("ERROR: Unrecognized message type: %u\n",
             entry->msg.nlmsg_type);
        ret = -EIO;
        break;
    }

  kmm_free(entry);
  return ret;
}

#endif /* CONFIG_NETLINK_CRYPTO */
