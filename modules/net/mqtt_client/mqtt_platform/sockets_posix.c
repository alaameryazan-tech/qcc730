/*
 */

/*
 * AWS IoT Device SDK for Embedded C 202211.00
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/* Standard includes. */
#include <assert.h>
#include <string.h>

/* POSIX sockets includes. */
#include <errno.h>
#include <netdb.h>
#include <time.h>
#include <unistd.h>
#include <sys/time.h>
//#include "lwip/inet.h"
#include <arpa/inet.h>
//#include "lwip/sockets.h"
#include <sys/socket.h>

#include "sockets_posix.h"

/*-----------------------------------------------------------*/

/**
 * @brief Number of milliseconds in one second.
 */
#define ONE_SEC_TO_MS (1000)

/**
 * @brief Number of microseconds in one millisecond.
 */
#define ONE_MS_TO_US (1000)

/**
 * @brief Bound on the TCP connect handshake specifically (connectToAddress()),
 * kept deliberately separate from the caller's sendTimeoutMs/recvTimeoutMs
 * (which only govern SO_SNDTIMEO/RCVTIMEO for sends/receives on an already-
 * connected socket, further down in Sockets_Connect()).
 *
 * Callers in this codebase generally pass a short (~1000ms) transport
 * timeout meant to catch a dead *established* connection quickly. Applying
 * that same short bound to the connect handshake itself turned out to be
 * too tight in practice on this platform: observed failures logged
 * SOCKETS_CONNECT_FAILURE almost exactly ~1000ms after the connect attempt
 * started - the handshake was still in progress (SYN sent, no response yet)
 * and got cut off, particularly right after the WLAN radio had just come
 * out of a BMPS/DTIM10 sleep-entry hiccup (nt_socpm_check_sleep_entry_failure
 * "sleep failed" in the platform's WLAN driver) and needed more than 1s to
 * fully recover before it could complete a handshake. A longer, fixed
 * budget here gives the handshake a fair chance without loosening the
 * failure-detection speed for already-established connections.
 */
#define SOCKETS_CONNECT_TIMEOUT_MS (5000U)

/*-----------------------------------------------------------*/

/**
 * @brief Resolve a host name.
 *
 * @param[in] pHostName Server host name.
 * @param[in] hostNameLength Length associated with host name.
 * @param[out] pListHead The output parameter to return the list containing
 * resolved DNS records.
 *
 * @return #SOCKETS_SUCCESS if successful; #SOCKETS_DNS_FAILURE, #SOCKETS_CONNECT_FAILURE on error.
 */
static SocketStatus_t resolveHostName(const char *pHostName, size_t hostNameLength, struct addrinfo **pListHead);

/**
 * @brief Traverse list of DNS records until a connection is established.
 *
 * @param[in] pListHead List containing resolved DNS records.
 * @param[in] pHostName Server host name.
 * @param[in] hostNameLength Length associated with host name.
 * @param[in] port Server port in host-order.
 * @param[in] connectTimeoutMs Bound on how long the connect handshake itself
 * (not send/recv afterward) may block - see connectToAddress().
 * @param[out] pTcpSocket The output parameter to return the created socket.
 *
 * @return #SOCKETS_SUCCESS if successful; #SOCKETS_CONNECT_FAILURE on error.
 */
static SocketStatus_t attemptConnection(struct addrinfo *pListHead, const char *pHostName, size_t hostNameLength,
                                        uint16_t port, uint32_t connectTimeoutMs, int32_t *pTcpSocket);

/**
 * @brief Connect to server using the provided address record.
 *
 * @param[in, out] pAddrInfo Address record of the server.
 * @param[in] port Server port in host-order.
 * @param[in] tcpSocket Socket handle.
 * @param[in] connectTimeoutMs How long to wait for the TCP handshake to
 * complete before giving up. The caller's sendTimeoutMs/recvTimeoutMs (SO_SND/
 * RCVTIMEO) only take effect once already connected - without this, connect()
 * itself is a fully blocking call bounded only by the OS/lwIP's own internal
 * SYN retry timeout, which can run for tens of seconds to minutes and was
 * silently ignoring the caller's intended timeout entirely.
 *
 * @return #SOCKETS_SUCCESS if successful; #SOCKETS_CONNECT_FAILURE on error.
 */
static SocketStatus_t connectToAddress(struct sockaddr *pAddrInfo, uint16_t port, int32_t tcpSocket,
                                       uint32_t connectTimeoutMs);

/**
 * @brief Log possible error using errno and return appropriate status.
 *
 * @param[in] errorNumber Error number.
 *
 * @return #SOCKETS_API_ERROR, #SOCKETS_INSUFFICIENT_MEMORY, #SOCKETS_INVALID_PARAMETER on error.
 */
static SocketStatus_t retrieveError(int32_t errorNumber);

/*-----------------------------------------------------------*/

static SocketStatus_t resolveHostName(const char *pHostName, size_t hostNameLength, struct addrinfo **pListHead)
{
    SocketStatus_t returnStatus = SOCKETS_SUCCESS;
    int32_t dnsStatus = -1;
    struct addrinfo hints;

    assert(pHostName != NULL);
    assert(hostNameLength > 0);

    /* Unused parameter. These parameters are used only for logging. */
    (void)hostNameLength;

    /* Add hints to retrieve only TCP sockets in getaddrinfo. */
    (void)memset(&hints, 0, sizeof(hints));

    /* Address family of either IPv4 or IPv6. */
    hints.ai_family = AF_UNSPEC;
    /* TCP Socket. */
    hints.ai_socktype = (int32_t)SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    /* Perform a DNS lookup on the given host name. */
    dnsStatus = getaddrinfo(pHostName, NULL, &hints, pListHead);

    if (dnsStatus != 0) {
        LogError(
            ("Failed to resolve DNS: Hostname=%.*s, ErrorCode=%d.\n", (int32_t)hostNameLength, pHostName, dnsStatus));
        returnStatus = SOCKETS_DNS_FAILURE;
    }

    return returnStatus;
}
/*-----------------------------------------------------------*/

static SocketStatus_t connectToAddress(struct sockaddr *pAddrInfo, uint16_t port, int32_t tcpSocket,
                                       uint32_t connectTimeoutMs)
{
    SocketStatus_t returnStatus = SOCKETS_SUCCESS;
    int32_t connectStatus = 0;
    int32_t nonBlockingFlag = 1;
#if LWIP_IPV6
    char resolvedIpAddr[INET6_ADDRSTRLEN];
#else
    char resolvedIpAddr[INET_ADDRSTRLEN];
#endif
    socklen_t addrInfoLength;
    uint16_t netPort = 0;
    struct sockaddr_in *pIpv4Address;
    struct sockaddr_in6 *pIpv6Address;

    assert(pAddrInfo != NULL);
    assert(pAddrInfo->sa_family == AF_INET || pAddrInfo->sa_family == AF_INET6);
    assert(tcpSocket >= 0);

    /* Convert port from host byte order to network byte order. */
    netPort = htons(port);

    if (pAddrInfo->sa_family == (sa_family_t)AF_INET) {
#if LWIP_IPV4
        /* MISRA Rule 11.3 flags the following line for casting a pointer of
         * a object type to a pointer of a different object type. This rule
         * is suppressed because casting from a struct sockaddr pointer to
         * a struct sockaddr_in pointer is supported in POSIX and is used
         * to obtain the IP address from the address record. */
        /* coverity[misra_c_2012_rule_11_3_violation] */
        pIpv4Address = (struct sockaddr_in *)pAddrInfo;
        /* Store IPv4 in string to log. */
        pIpv4Address->sin_port = netPort;
        addrInfoLength = (socklen_t)sizeof(struct sockaddr_in);
        (void)inet_ntop((int32_t)pAddrInfo->sa_family, &pIpv4Address->sin_addr, resolvedIpAddr,
                        (socklen_t)sizeof(resolvedIpAddr));
#else
        (void)closesocket(tcpSocket);
        return SOCKETS_CONNECT_FAILURE;
#endif
    } else {
#if LWIP_IPV6
        /* MISRA Rule 11.3 flags the following line for casting a pointer of
         * a object type to a pointer of a different object type. This rule
         * is suppressed because casting from a struct sockaddr pointer to
         * a struct sockaddr_in6 pointer is supported in POSIX and is used
         * to obtain the IPv6 address from the address record. */
        /* coverity[misra_c_2012_rule_11_3_violation] */
        pIpv6Address = (struct sockaddr_in6 *)pAddrInfo;
        /* Store IPv6 in string to log. */
        pIpv6Address->sin6_port = netPort;
        addrInfoLength = (socklen_t)sizeof(struct sockaddr_in6);
        (void)inet_ntop((int32_t)pAddrInfo->sa_family, &pIpv6Address->sin6_addr, resolvedIpAddr,
                        (socklen_t)sizeof(resolvedIpAddr));
#else
        (void)closesocket(tcpSocket);
        return SOCKETS_CONNECT_FAILURE;
#endif
    }

    LogDebug(
        ("Attempting to connect to server using the resolved IP address:"
         " IP address=%s.",
         resolvedIpAddr));

    /* Make the socket non-blocking for the connect handshake specifically,
     * so a stalled SYN (no route, lost packet, AP-side power-save hiccup)
     * is bounded by connectTimeoutMs instead of the OS/lwIP's own internal
     * SYN retry timeout - previously connect() below was fully blocking
     * and the caller's timeout was never applied to it at all, only to
     * sends/receives after the fact. Restored to blocking afterward, since
     * the rest of this file (and Plaintext_Send/Recv) assumes a blocking
     * socket with SO_SNDTIMEO/RCVTIMEO, set by the caller right after this
     * returns.
     *
     * Uses ioctl(FIONBIO) rather than fcntl(O_NONBLOCK): this codebase has
     * more than one fcntl.h in its include paths (e.g. modules/fs/fcntl.h)
     * defining O_NONBLOCK as a different bit value than lwIP's own, so
     * which one wins depends on include-path resolution - a real risk of
     * silently not actually setting non-blocking mode. FIONBIO is the
     * mechanism lwIP's own sockets.h documents as supported - via
     * ioctlsocket(), not the bare ioctl() name: that one's only mapped to
     * lwip_ioctl() when LWIP_POSIX_SOCKETS_IO_NAMES is set, which this
     * build's lwipopts doesn't enable (confirmed by the linker: undefined
     * reference to `ioctl`). ioctlsocket() is defined unconditionally
     * alongside closesocket()/connect()/etc. below that guard, so it's the
     * name actually available here. */
    connectStatus = ioctlsocket(tcpSocket, FIONBIO, &nonBlockingFlag);
    if (connectStatus != 0) {
        LogError(("Failed to set socket non-blocking before connect: IP address=%s.", resolvedIpAddr));
        (void)closesocket(tcpSocket);
        return SOCKETS_CONNECT_FAILURE;
    }

    connectStatus = connect(tcpSocket, pAddrInfo, addrInfoLength);

    if (connectStatus == -1) {
        int32_t connectErrno = errno;

        if ((connectErrno == EINPROGRESS) || (connectErrno == EWOULDBLOCK)) {
            fd_set writeFds;
            struct timeval connectTimeout;
            int32_t selectStatus;

            FD_ZERO(&writeFds);
            FD_SET(tcpSocket, &writeFds);
            connectTimeout.tv_sec = (((int64_t)connectTimeoutMs) / ONE_SEC_TO_MS);
            connectTimeout.tv_usec = (ONE_MS_TO_US * (((int64_t)connectTimeoutMs) % ONE_SEC_TO_MS));

            selectStatus = select(tcpSocket + 1, NULL, &writeFds, NULL, &connectTimeout);

            if (selectStatus > 0) {
                int32_t sockErr = 0;
                socklen_t sockErrLen = (socklen_t)sizeof(sockErr);

                if ((getsockopt(tcpSocket, SOL_SOCKET, SO_ERROR, &sockErr, &sockErrLen) == 0) && (sockErr == 0)) {
                    connectStatus = 0;
                } else {
                    /* printf(), not LogError() - LogError from this file has
                     * never once shown up on the actual console across a
                     * long debugging session, so it's not routed anywhere
                     * visible here. This is the one piece of information
                     * that was missing to tell "SYN sent, never answered"
                     * apart from "handshake completed but with an error"
                     * apart from "rejected immediately, never even tried". */
                    printf("Sockets_Connect: connect() completed with error after select(), SO_ERROR=%d (%s): IP address=%s\r\n",
                           (int)sockErr, strerror(sockErr), resolvedIpAddr);
                    connectStatus = -1;
                }
            } else {
                /* selectStatus == 0: timed out without the handshake settling.
                 * selectStatus < 0: select() itself failed. Either way the
                 * connect didn't complete in time. */
                printf("Sockets_Connect: connect() timed out after %u ms waiting for handshake (selectStatus=%d): IP address=%s\r\n",
                       (unsigned int)connectTimeoutMs, (int)selectStatus, resolvedIpAddr);
                connectStatus = -1;
            }
        } else {
            /* connect() rejected the attempt immediately - it never even
             * got as far as sending a SYN over the air. This is the case
             * that was completely invisible before: no route to host, ARP
             * failure, network unreachable, etc. all landed on the exact
             * same unlabeled "Plaintext_Connect failed, status=5" as a
             * genuine handshake timeout, with nothing distinguishing them. */
            printf("Sockets_Connect: connect() rejected immediately, errno=%d (%s): IP address=%s\r\n",
                   (int)connectErrno, strerror(connectErrno), resolvedIpAddr);
        }
    }

    /* Always restore blocking mode before returning, on both the success
     * and failure paths - a closed socket just ignores the ioctlsocket() call. */
    nonBlockingFlag = 0;
    (void)ioctlsocket(tcpSocket, FIONBIO, &nonBlockingFlag);

    if (connectStatus == -1) {
        LogError(("Failed to connect to server using the resolved IP address: IP address=%s.", resolvedIpAddr));
        (void)closesocket(tcpSocket);
        returnStatus = SOCKETS_CONNECT_FAILURE;
    }

    return returnStatus;
}
/*-----------------------------------------------------------*/

static SocketStatus_t attemptConnection(struct addrinfo *pListHead, const char *pHostName, size_t hostNameLength,
                                        uint16_t port, uint32_t connectTimeoutMs, int32_t *pTcpSocket)
{
    SocketStatus_t returnStatus = SOCKETS_CONNECT_FAILURE;
    const struct addrinfo *pIndex = NULL;

    assert(pListHead != NULL);
    assert(pHostName != NULL);
    assert(hostNameLength > 0);
    assert(pTcpSocket != NULL);

    /* Unused parameters when logging is disabled. */
    (void)pHostName;
    (void)hostNameLength;

    LogDebug(("Attempting to connect to: Host=%.*s.", (int32_t)hostNameLength, pHostName));

    /* Attempt to connect to one of the retrieved DNS records. */
    for (pIndex = pListHead; pIndex != NULL; pIndex = pIndex->ai_next) {
        *pTcpSocket = socket(pIndex->ai_family, pIndex->ai_socktype, pIndex->ai_protocol);

        if (*pTcpSocket == -1) {
            continue;
        }

        /* Attempt to connect to a resolved DNS address of the host. */
        returnStatus = connectToAddress(pIndex->ai_addr, port, *pTcpSocket, connectTimeoutMs);

        /* If connected to an IP address successfully, exit from the loop. */
        if (returnStatus == SOCKETS_SUCCESS) {
            break;
        }
    }

    if (returnStatus == SOCKETS_SUCCESS) {
        LogDebug(("Established TCP connection: Server=%.*s.\n", (int32_t)hostNameLength, pHostName));
    } else {
        LogError(("Could not connect to any resolved IP address from %.*s.", (int32_t)hostNameLength, pHostName));
        *pTcpSocket = -1;
    }

    freeaddrinfo(pListHead);

    return returnStatus;
}
/*-----------------------------------------------------------*/

static SocketStatus_t retrieveError(int32_t errorNumber)
{
    SocketStatus_t returnStatus = SOCKETS_API_ERROR;

    LogError(("A transport error occured: %s.", strerror(errorNumber)));

    if ((errorNumber == ENOMEM) || (errorNumber == ENOBUFS)) {
        returnStatus = SOCKETS_INSUFFICIENT_MEMORY;
    } else if ((errorNumber == ENOTSOCK) || (errorNumber == EDOM) || (errorNumber == EBADF)) {
        returnStatus = SOCKETS_INVALID_PARAMETER;
    } else {
        /* Empty else. */
    }

    return returnStatus;
}
/*-----------------------------------------------------------*/

SocketStatus_t Sockets_Connect(int32_t *pTcpSocket, const ServerInfo_t *pServerInfo, uint32_t sendTimeoutMs,
                               uint32_t recvTimeoutMs)
{
    SocketStatus_t returnStatus = SOCKETS_SUCCESS;
    struct addrinfo *pListHead = NULL;
    struct timeval transportTimeout;
    int32_t setTimeoutStatus = -1;

    if (pServerInfo == NULL) {
        LogError(("Parameter check failed: pServerInfo is NULL."));
        returnStatus = SOCKETS_INVALID_PARAMETER;
    } else if (pServerInfo->pHostName == NULL) {
        LogError(("Parameter check failed: pServerInfo->pHostName is NULL."));
        returnStatus = SOCKETS_INVALID_PARAMETER;
    } else if (pTcpSocket == NULL) {
        LogError(("Parameter check failed: pTcpSocket is NULL."));
        returnStatus = SOCKETS_INVALID_PARAMETER;
    } else if (pServerInfo->hostNameLength == 0UL) {
        LogError(("Parameter check failed: hostNameLength must be greater than 0."));
        returnStatus = SOCKETS_INVALID_PARAMETER;
    } else {
        /* Empty else. */
    }

    if (returnStatus == SOCKETS_SUCCESS) {
        returnStatus = resolveHostName(pServerInfo->pHostName, pServerInfo->hostNameLength, &pListHead);
    }

    if (returnStatus == SOCKETS_SUCCESS) {
        /* The connect handshake itself is now bounded (see
         * connectToAddress()) - previously it was fully unbounded, blocked
         * only by the OS/lwIP's own internal SYN retry timeout, silently
         * ignoring any timeout the caller thought applied. Uses a fixed,
         * more generous SOCKETS_CONNECT_TIMEOUT_MS rather than the
         * caller's sendTimeoutMs/recvTimeoutMs - those two continue to
         * mean exactly what they did before (SO_SNDTIMEO/RCVTIMEO for an
         * already-connected socket, below), unaffected by this. */
        returnStatus = attemptConnection(pListHead, pServerInfo->pHostName, pServerInfo->hostNameLength,
                                         pServerInfo->port, SOCKETS_CONNECT_TIMEOUT_MS, pTcpSocket);
    }

    /* Set the send timeout. */
    if (returnStatus == SOCKETS_SUCCESS) {
#if LWIP_SO_SNDRCVTIMEO_NONSTANDARD
        transportTimeout = sendTimeoutMs;
#else
        transportTimeout.tv_sec = (((int64_t)sendTimeoutMs) / ONE_SEC_TO_MS);
        transportTimeout.tv_usec = (ONE_MS_TO_US * (((int64_t)sendTimeoutMs) % ONE_SEC_TO_MS));
#endif
        setTimeoutStatus =
            setsockopt(*pTcpSocket, SOL_SOCKET, SO_SNDTIMEO, &transportTimeout, (socklen_t)sizeof(transportTimeout));

        if (setTimeoutStatus < 0) {
            if (errno == ENOPROTOOPT) {
                LogInfo(("Setting socket send timeout skipped."));
            } else {
                LogError(("Setting socket send timeout failed."));
                returnStatus = retrieveError(errno);
            }
        }
    }

    /* Set the receive timeout. */
    if (returnStatus == SOCKETS_SUCCESS) {
        transportTimeout.tv_sec = (((int64_t)recvTimeoutMs) / ONE_SEC_TO_MS);
        transportTimeout.tv_usec = (ONE_MS_TO_US * (((int64_t)recvTimeoutMs) % ONE_SEC_TO_MS));

        setTimeoutStatus =
            setsockopt(*pTcpSocket, SOL_SOCKET, SO_RCVTIMEO, &transportTimeout, (socklen_t)sizeof(transportTimeout));

        if (setTimeoutStatus < 0) {
            if (errno == ENOPROTOOPT) {
                LogInfo(("Setting socket receive timeout skipped."));
            } else {
                LogError(("Setting socket receive timeout failed."));
                returnStatus = retrieveError(errno);
            }
        }
    }

    return returnStatus;
}
/*-----------------------------------------------------------*/

SocketStatus_t Sockets_Disconnect(int32_t tcpSocket)
{
    SocketStatus_t returnStatus = SOCKETS_SUCCESS;

    if (tcpSocket >= 0) {
        (void)shutdown(tcpSocket, SHUT_RDWR);
        (void)closesocket(tcpSocket);
    } else {
        returnStatus = SOCKETS_INVALID_PARAMETER;
    }

    return returnStatus;
}
/*-----------------------------------------------------------*/
