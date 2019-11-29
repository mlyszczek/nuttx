/****************************************************************************
 * include/nuttx/crypto/crypto_netlink.h
 * Describes Netlink interface between the socket layer and the crypto
 * subsystem.
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

#ifndef __INCLUDE_NUTTX_CRYPTO_CRYPTO_NETLINK_H
#define __INCLUDE_NUTTX_CRYPTO_CRYPTO_NETLINK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <netpacket/netlink.h>
#include <nuttx/net/netlink.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: crypto_request_handler
 *
 * Description:
 *   This function receives the raw NETLINK_CRYPTO request and provides that
 *   request to the appropriate handler.
 *
 * Input Parameters:
 *   handle - A handle for use in subsequent calls to netlink_add_response()
 *            when the response to the request is available.
 *   req    - A reference to the common header of the Netlink messages.
 *            This may be cast to the specific message type associated with
 *            the decoded Netlink messages.
 *   reqlen - The full length of the request 'req'
 *   to     - The destination address for bound sockets.
 *
 * Returned Value
 *   Zero is returned on success meaning that the response was successfully
 *   processed.  In this case, a response has been or will be generated.  In
 *   the event of an error, a negated errno value will be returned.
 *
 ****************************************************************************/

int crypto_request_handler(NETLINK_HANDLE handle,
                           FAR const struct nlmsghdr *req, size_t reqlen,
                           FAR const struct sockaddr_alg *to);

#endif /* __INCLUDE_NUTTX_CRYPTO_CRYPTO_NETLINK_H */
