/*
 * linux/can.hpp
 *
 * Definitions for CAN network layer (socket addr / CAN frame / CAN filter)
 *
 * Authors: Oliver Hartkopp <oliver.hartkopp@volkswagen.de>
 *          Urs Thuermann   <urs.thuermann@volkswagen.de>
 * Copyright (c) 2002-2007 Volkswagen Group Electronic Research
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of Volkswagen nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * Alternatively, provided that this notice is retained in full, this
 * software may be distributed under the terms of the GNU General
 * Public License ("GPL") version 2, in which case the provisions of the
 * GPL apply INSTEAD OF those given above.
 *
 * The provided data structures and external interfaces from this code
 * are not restricted to be used by modules with a GPL compatible license.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */

//-----------------------------------------------------------------------------//
// Modified by Laurent Lardinois for ESP32 and Linux compilation               //
// Laurent Lardinois https://be.linkedin.com/in/laurentlardinois               //
//                                                                             //
// https://github.com/type-one/can-utils-esp32                                 //
//-----------------------------------------------------------------------------// 

#ifndef CAN_KERNEL_H
#define CAN_KERNEL_H

#include <cstdint>

/* controller area network (CAN) kernel definitions */

/* special address description flags for the CAN_ID */
#define CAN_EFF_FLAG 0x80000000U /* EFF/SFF is set in the MSB */
#define CAN_RTR_FLAG 0x40000000U /* remote transmission request */
#define CAN_ERR_FLAG 0x20000000U /* error message frame */

/* valid bits in CAN ID for frame formats */
#define CAN_SFF_MASK 0x000007FFU /* standard frame format (SFF) */
#define CAN_EFF_MASK 0x1FFFFFFFU /* extended frame format (EFF) */
#define CAN_ERR_MASK 0x1FFFFFFFU /* omit EFF, RTR, ERR flags */

/*
 * Controller Area Network Identifier structure
 *
 * bit 0-28	: CAN identifier (11/29 bit)
 * bit 29	: error message frame flag (0 = data frame, 1 = error message)
 * bit 30	: remote transmission request flag (1 = rtr frame)
 * bit 31	: frame format flag (0 = standard 11 bit, 1 = extended 29 bit)
 */
typedef std::uint32_t canid_t;

#define CAN_SFF_ID_BITS 11
#define CAN_EFF_ID_BITS 29

/*
 * Controller Area Network Error Message Frame Mask structure
 *
 * bit 0-28	: error class mask (see include/uapi/linux/can/error.h)
 * bit 29-31	: set to zero
 */
typedef std::uint32_t can_err_mask_t;

/* CAN payload length and DLC definitions according to ISO 11898-1 */
#define CAN_MAX_DLC 8
#define CAN_MAX_RAW_DLC 15
#define CAN_MAX_DLEN 8

/* CAN FD payload length and DLC definitions according to ISO 11898-7 */
#define CANFD_MAX_DLC 15
#define CANFD_MAX_DLEN 64

/**
 * struct can_frame - Classical CAN frame structure (aka CAN 2.0B)
 * @can_id:   CAN ID of the frame and CAN_*_FLAG flags, see canid_t definition
 * @len:      CAN frame payload length in byte (0 .. 8)
 * @can_dlc:  deprecated name for CAN frame payload length in byte (0 .. 8)
 * @__pad:    padding
 * @__res0:   reserved / padding
 * @len8_dlc: optional DLC value (9 .. 15) at 8 byte payload length
 *            len8_dlc contains values from 9 .. 15 when the payload length is
 *            8 bytes but the DLC value (see ISO 11898-1) is greater then 8.
 *            CAN_CTRLMODE_CC_LEN8_DLC flag has to be enabled in CAN driver.
 * @data:     CAN frame payload (up to 8 byte)
 */
struct can_frame
{
    canid_t can_id; /* 32 bit CAN_ID + EFF/RTR/ERR flags */
#ifdef _MSC_VER
    __pragma(pack(push, 1))
#endif // _MSC_VER
        union
    {
        /* CAN frame payload length in byte (0 .. CAN_MAX_DLEN)
         * was previously named can_dlc so we need to carry that
         * name for legacy support
         */
        std::uint8_t len;
        std::uint8_t can_dlc; /* deprecated */
    }
#ifndef _MSC_VER
    __attribute__((packed)) /* disable padding added in some ABIs */
#endif                      // _MSC_VER
    ;
#ifdef _MSC_VER
    __pragma(pack(pop))
#endif // _MSC_VER

        std::uint8_t __pad; /* padding */
    std::uint8_t __res0;    /* reserved / padding */
    std::uint8_t len8_dlc;  /* optional DLC for 8 byte payload length (9 .. 15) */
#ifdef _MSC_VER
    __declspec(align(8)) __u8 data[CAN_MAX_DLEN];
#else  // _MSC_VER
    std::uint8_t data[CAN_MAX_DLEN] __attribute__((aligned(8)));
#endif // _MSC_VER
};

/*
 * defined bits for canfd_frame.flags
 *
 * The use of struct canfd_frame implies the FD Frame (FDF) bit to
 * be set in the CAN frame bitstream on the wire. The FDF bit switch turns
 * the CAN controllers bitstream processor into the CAN FD mode which creates
 * two new options within the CAN FD frame specification:
 *
 * Bit Rate Switch - to indicate a second bitrate is/was used for the payload
 * Error State Indicator - represents the error state of the transmitting node
 *
 * As the CANFD_ESI bit is internally generated by the transmitting CAN
 * controller only the CANFD_BRS bit is relevant for real CAN controllers when
 * building a CAN FD frame for transmission. Setting the CANFD_ESI bit can make
 * sense for virtual CAN interfaces to test applications with echoed frames.
 *
 * The struct can_frame and struct canfd_frame intentionally share the same
 * layout to be able to write CAN frame content into a CAN FD frame structure.
 * When this is done the former differentiation via CAN_MTU / CANFD_MTU gets
 * lost. CANFD_FDF allows programmers to mark CAN FD frames in the case of
 * using struct canfd_frame for mixed CAN / CAN FD content (dual use).
 * N.B. the Kernel APIs do NOT provide mixed CAN / CAN FD content inside of
 * struct canfd_frame therefore the CANFD_FDF flag is disregarded by Linux.
 */
#define CANFD_BRS 0x01 /* bit rate switch (second bitrate for payload data) */
#define CANFD_ESI 0x02 /* error state indicator of the transmitting node */
#define CANFD_FDF 0x04 /* mark CAN FD for dual use of struct canfd_frame */

/**
 * struct canfd_frame - CAN flexible data rate frame structure
 * @can_id: CAN ID of the frame and CAN_*_FLAG flags, see canid_t definition
 * @len:    frame payload length in byte (0 .. CANFD_MAX_DLEN)
 * @flags:  additional flags for CAN FD
 * @__res0: reserved / padding
 * @__res1: reserved / padding
 * @data:   CAN FD frame payload (up to CANFD_MAX_DLEN byte)
 */
struct canfd_frame
{
    canid_t can_id;      /* 32 bit CAN_ID + EFF/RTR/ERR flags */
    std::uint8_t len;    /* frame payload length in byte */
    std::uint8_t flags;  /* additional flags for CAN FD */
    std::uint8_t __res0; /* reserved / padding */
    std::uint8_t __res1; /* reserved / padding */
#ifdef _MSC_VER
    __declspec(align(8)) std::uint8_t data[CANFD_MAX_DLEN];
#else  // _MSC_VER
    std::uint8_t data[CANFD_MAX_DLEN] __attribute__((aligned(8)));
#endif // _MSC_VER
};

#define CAN_MTU (sizeof(struct can_frame))
#define CANFD_MTU (sizeof(struct canfd_frame))

/* particular protocols of the protocol family PF_CAN */
#define CAN_RAW 1   /* RAW sockets */
#define CAN_BCM 2   /* Broadcast Manager */
#define CAN_TP16 3  /* VAG Transport Protocol v1.6 */
#define CAN_TP20 4  /* VAG Transport Protocol v2.0 */
#define CAN_MCNET 5 /* Bosch MCNet */
#define CAN_ISOTP 6 /* ISO 15765-2 Transport Protocol */
#define CAN_J1939 7 /* SAE J1939 */
#define CAN_NPROTO 8

#define SOL_CAN_BASE 100

/*
 * This typedef was introduced in Linux v3.1-rc2
 * (commit 6602a4b net: Make userland include of netlink.h more sane)
 * in <linux/socket.h>. It must be duplicated here to make the CAN
 * headers self-contained.
 */
typedef unsigned short __kernel_sa_family_t;

/**
 * struct sockaddr_can - the sockaddr structure for CAN sockets
 * @can_family:  address family number AF_CAN.
 * @can_ifindex: CAN network interface index.
 * @can_addr:    protocol specific address information
 */
struct sockaddr_can
{
    __kernel_sa_family_t can_family;
    int can_ifindex;
    union
    {
        /* transport protocol class address information (e.g. ISOTP) */
        struct
        {
            canid_t rx_id, tx_id;
        } tp;

        /* J1939 address information */
        struct
        {
            /* 8 byte name when using dynamic addressing */
            std::uint64_t name;

            /* pgn:
             * 8 bit: PS in PDU2 case, else 0
             * 8 bit: PF
             * 1 bit: DP
             * 1 bit: reserved
             */
            std::uint32_t pgn;

            /* 1 byte address */
            std::uint8_t addr;
        } j1939;

        /* reserved for future CAN protocols address information */
    } can_addr;
};

/**
 * struct can_filter - CAN ID based filter in can_register().
 * @can_id:   relevant bits of CAN ID which are not masked out.
 * @can_mask: CAN mask (see description)
 *
 * Description:
 * A filter matches, when
 *
 *          <received_can_id> & mask == can_id & mask
 *
 * The filter can be inverted (CAN_INV_FILTER bit set in can_id) or it can
 * filter for error message frames (CAN_ERR_FLAG bit set in mask).
 */
struct can_filter
{
    canid_t can_id;
    canid_t can_mask;
};

#define CAN_INV_FILTER 0x20000000U /* to be set in can_filter.can_id */
#define CAN_RAW_FILTER_MAX 512     /* maximum number of can_filter set via setsockopt() */

#endif /* CAN_KERNEL_H */
