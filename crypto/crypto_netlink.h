/****************************************************************************
 * crypto/crypto_netlink.h
 * Describes NETLINK_CRYPTO messaging
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __CRYPTO_CRYPTO_NETLINK_H
#define __CRYPTO_CRYPTO_NETLINK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <netpacket/netlink.h>
#include <nuttx/net/netlink.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Request Message Structures */

struct crypto_msg_newalg_request_s
{
  struct nlmsghdr        hdr;    /* nlmsg_type = CRYPTO_MSG_NEWALG */
  struct crypto_user_alg alg;
};

struct crypto_msg_delalg_request_s
{
  struct nlmsghdr        hdr;    /* nlmsg_type = CRYPTO_MSG_DELALG */
  struct crypto_user_alg alg;
};

struct crypto_msg_updatealg_request_s
{
  struct nlmsghdr        hdr;    /* nlmsg_type = CRYPTO_MSG_UPDATEALG */
  struct crypto_user_alg alg;
};

struct crypto_msg_getalg_request_s
{
  struct nlmsghdr        hdr;    /* nlmsg_type = CRYPTO_MSG_GETALG */
  struct crypto_user_alg alg;
};

struct crypto_msg_delrng_request_s
{
  struct nlmsghdr        hdr;    /* nlmsg_type = CRYPTO_MSG_DELRNG */
};

struct crypto_msg_getstat_request_s
{
  struct nlmsghdr        hdr;    /* nlmsg_type = CRYPTO_MSG_GETSTAT */
  struct crypto_user_alg alg;
};

/* Response message structures */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: crypto_*alg_request
 *
 * Description:
 *   The is a set of messages handlers that are required to provide the
 *   response to each NETLINK_CRYPTO request message.
 *
 *   If successfully, each function will create the appropriate response
 *   message and queue that message via a call to netlink_add_response().
 *   That call may be synchronous with the call to the response handler
 *   or may occur sometime later asynchronously.
 *
 * Input Parameters:
 *   handle - A handle for use in subsequent calls to netlink_add_response()
 *            when the response to the request is available.
 *   req    - A reference received request.
 *   reqlen - The size of the received request.  This size is guaranteed by
 *            the message dispatcher to be at least as large as the type
 *            refereed to by the 'req' parameter.
 *
 * Returned Value
 *   Zero is returned on success meaning that the response was successfully
 *   processed.  In this case, a response has been or will be generated.  In
 *   the event of an error, a negated errno value will be returned.
 *
 ****************************************************************************/

int crypto_newalg_request(NETLINK_HANDLE handle,
                          FAR const struct crypto_msg_newalg_request_s *req,
                          size_t reqlen, FAR const struct sockaddr_alg *to);
int crypto_delalg_request(NETLINK_HANDLE handle,
                          FAR const struct crypto_msg_delalg_request_s *req,
                          size_t reqlen, FAR const struct sockaddr_alg *to);
int crypto_updatealg_request(NETLINK_HANDLE handle,
                             FAR const struct crypto_msg_updatealg_request_s *req,
                             size_t reqlen, FAR const struct sockaddr_alg *to);
int crypto_getalg_request(NETLINK_HANDLE handle,
                          FAR const struct crypto_msg_getalg_request_s *req,
                          size_t reqlen, FAR const struct sockaddr_alg *to);
int crypto_delrng_request(NETLINK_HANDLE handle,
                          FAR const struct crypto_msg_delrng_request_s *req,
                          size_t reqlen, FAR const struct sockaddr_alg *to);
int crypto_getstat_request(NETLINK_HANDLE handle,
                           FAR const struct crypto_msg_getstat_request_s *req,
                           size_t reqlen, FAR const struct sockaddr_alg *to);

#endif /* __CRYPTO_CRYPTO_NETLINK_H */
