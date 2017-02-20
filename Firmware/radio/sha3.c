/* -------------------------------------------------------------------------
 * Works when compiled for either 32-bit or 64-bit targets, optimized for 
 * 64 bit.
 *
 * Canonical implementation of Init/Update/Finalize for SHA-3 byte input. 
 *
 * SHA3-256, SHA3-384, SHA-512 are implemented. SHA-224 can easily be added.
 *
 * Based on code from http://keccak.noekeon.org/ .
 *
 * I place the code that I wrote into public domain, free to use. 
 *
 * I would appreciate if you give credits to this work if you used it to 
 * write or test * your code.
 *
 * Aug 2015. Andrey Jivsov. crypto@brainhub.org
 * ---------------------------------------------------------------------- */

#include "sha3.h"

static const __code uint64_t keccakf_rndc[24] = {
    SHA3_CONST(0x0000000000000001UL), SHA3_CONST(0x0000000000008082UL),
    SHA3_CONST(0x800000000000808aUL), SHA3_CONST(0x8000000080008000UL),
    SHA3_CONST(0x000000000000808bUL), SHA3_CONST(0x0000000080000001UL),
    SHA3_CONST(0x8000000080008081UL), SHA3_CONST(0x8000000000008009UL),
    SHA3_CONST(0x000000000000008aUL), SHA3_CONST(0x0000000000000088UL),
    SHA3_CONST(0x0000000080008009UL), SHA3_CONST(0x000000008000000aUL),
    SHA3_CONST(0x000000008000808bUL), SHA3_CONST(0x800000000000008bUL),
    SHA3_CONST(0x8000000000008089UL), SHA3_CONST(0x8000000000008003UL),
    SHA3_CONST(0x8000000000008002UL), SHA3_CONST(0x8000000000000080UL),
    SHA3_CONST(0x000000000000800aUL), SHA3_CONST(0x800000008000000aUL),
    SHA3_CONST(0x8000000080008081UL), SHA3_CONST(0x8000000000008080UL),
    SHA3_CONST(0x0000000080000001UL), SHA3_CONST(0x8000000080008008UL)
};

static const __code uint32_t keccakf_rotc[24] = {
    1, 3, 6, 10, 15, 21, 28, 36, 45, 55, 2, 14, 27, 41, 56, 8, 25, 43, 62,
    18, 39, 61, 20, 44
};

static const __code uint32_t keccakf_piln[24] = {
    10, 7, 11, 17, 18, 3, 5, 16, 8, 21, 24, 4, 15, 23, 19, 13, 12, 2, 20,
    14, 22, 9, 6, 1
};

__xdata struct sha3_context ctx;

/* generally called after SHA3_KECCAK_SPONGE_WORDS-ctx->capacityWords words 
 * are XORed into the state s 
 */
uint64_t t, bc[5];
uint8_t n, j, r, round;
static void keccakf(void)
{
#define KECCAK_ROUNDS 24

    for(round = 0; round < KECCAK_ROUNDS; round++) {
      /* Theta */
      for(n = 0; n < 5; n++) {
	bc[n] = ctx.s[n];
	bc[n]^=ctx.s[n + 5];
	bc[n]^=ctx.s[n + 10];
	bc[n]^=ctx.s[n + 15];
	bc[n]^=ctx.s[n + 20];
      }

        for(n = 0; n < 5; n++) {
	    t = bc[(n + 4) % 5] ^ SHA3_ROTL64(bc[(n + 1) % 5], 1);
            for(j = 0; j < 25; j += 5)
                ctx.s[j + n] ^= t;
        }

        /* Rho Pi */
        t = ctx.s[1];
        for(n = 0; n < 24; n++) {
	    j = keccakf_piln[n];
            bc[0] = ctx.s[j];	    
	    for (r=0;r<keccakf_rotc[n];r++) t=SHA3_ROTL64(t, 1);
	    ctx.s[j] = t;
            t = bc[0];
        }

        /* Chi */
        for(j = 0; j < 25; j += 5) {
            for(n = 0; n < 5; n++)
                bc[n] = ctx.s[j + n];
            for(n = 0; n < 5; n++)
                ctx.s[j + n] ^= (~bc[(n + 1) % 5]) & bc[(n + 2) % 5];
        }

        /* Iota */
        ctx.s[0] ^= keccakf_rndc[round];
    }
}

/* *************************** Public Inteface ************************ */

/* For Init or Reset call these: */
void sha3_Init256(void)
{
    memset(&ctx, 0, sizeof(ctx));
    ctx.capacityWords = 2 * 256 / (8 * sizeof(uint64_t));
}

void sha3_Init384(void)
{
    memset(&ctx, 0, sizeof(ctx));
    ctx.capacityWords = 2 * 384 / (8 * sizeof(uint64_t));
}

void sha3_Init512(void)
{
    memset(&ctx, 0, sizeof(ctx));
    ctx.capacityWords = 2 * 512 / (8 * sizeof(uint64_t));
}

__xdata uint32_t old_tail;
__xdata size_t words;
__xdata uint32_t tail;
__xdata size_t ii;
__xdata uint64_t t;
__xdata uint8_t *buf;

void sha3_Update(void *bufIn, size_t len)
{
    /* 0...7 -- how much is needed to have a word */
    old_tail = (8 - ctx.byteIndex) & 7;
    buf = bufIn;

    SHA3_TRACE_BUF("called to update with:", buf, len);

    SHA3_ASSERT(ctx.byteIndex < 8);
    SHA3_ASSERT(ctx.wordIndex < sizeof(ctx.s) / sizeof(ctx.s[0]));

    if(len < old_tail) {        /* have no complete word or haven't started 
                                 * the word yet */
        SHA3_TRACE("because %d<%d, store it and return", (unsigned)len,
                (unsigned)old_tail);
        /* endian-independent code follows: */
        while (len--)
            ctx.saved |= (uint64_t) (*(buf++)) << ((ctx.byteIndex++) * 8);
        SHA3_ASSERT(ctx.byteIndex < 8);
        return;
    }

    if(old_tail) {              /* will have one word to process */
        SHA3_TRACE("completing one word with %d bytes", (unsigned)old_tail);
        /* endian-independent code follows: */
        len -= old_tail;
        while (old_tail--)
            ctx.saved |= (uint64_t) (*(buf++)) << ((ctx.byteIndex++) * 8);

        /* now ready to add saved to the sponge */
        ctx.s[ctx.wordIndex] ^= ctx.saved;
        SHA3_ASSERT(ctx.byteIndex == 8);
        ctx.byteIndex = 0;
        ctx.saved = 0;
        if(++ctx.wordIndex ==
                (SHA3_KECCAK_SPONGE_WORDS - ctx.capacityWords)) {
            keccakf();
            ctx.wordIndex = 0;
        }
    }

    /* now work in full words directly from input */

    SHA3_ASSERT(ctx.byteIndex == 0);

    words = len / sizeof(uint64_t);
    tail = len - words * sizeof(uint64_t);

    SHA3_TRACE("have %d full words to process", (unsigned)words);

    for(ii = 0; ii < words; ii++, buf += sizeof(uint64_t)) {
        t = (uint64_t) (buf[0]) |
                ((uint64_t) (buf[1]) << 8 * 1) |
                ((uint64_t) (buf[2]) << 8 * 2) |
                ((uint64_t) (buf[3]) << 8 * 3) |
                ((uint64_t) (buf[4]) << 8 * 4) |
                ((uint64_t) (buf[5]) << 8 * 5) |
                ((uint64_t) (buf[6]) << 8 * 6) |
                ((uint64_t) (buf[7]) << 8 * 7);
#if defined(__x86_64__ ) || defined(__i386__)
        SHA3_ASSERT(memcmp(&t, buf, 8) == 0);
#endif
        ctx.s[ctx.wordIndex] ^= t;
        if(++ctx.wordIndex ==
                (SHA3_KECCAK_SPONGE_WORDS - ctx.capacityWords)) {
            keccakf();
            ctx.wordIndex = 0;
        }
    }

    SHA3_TRACE("have %d bytes left to process, save them", (unsigned)tail);

    /* finally, save the partial word */
    SHA3_ASSERT(ctx.byteIndex == 0 && tail < 8);
    while (tail--) {
        SHA3_TRACE("Store byte %02x '%c'", *buf, *buf);
        ctx.saved |= (uint64_t) (*(buf++)) << ((ctx.byteIndex++) * 8);
    }
    SHA3_ASSERT(ctx.byteIndex < 8);
    SHA3_TRACE("Have saved=0x%016" PRIx64 " at the end", ctx.saved);
}

/* This is simply the 'update' with the padding block.
 * The padding block is 0x01 || 0x00* || 0x80. First 0x01 and last 0x80 
 * bytes are always present, but they can be the same byte.
 */
__xdata uint32_t t1;
__xdata uint32_t t2;
__xdata uint8_t word;

void sha3_Finalize(void)
{
  
    SHA3_TRACE("called with %d bytes in the buffer", ctx.byteIndex);

    /* Append 2-bit suffix 01, per SHA-3 spec. Instead of 1 for padding we
     * use 1<<2 below. The 0x02 below corresponds to the suffix 01.
     * Overall, we feed 0, then 1, and finally 1 to start padding. Without
     * M || 01, we would simply use 1 to start padding. */

#ifndef SHA3_USE_KECCAK
    /* SHA3 version */
    ctx.s[ctx.wordIndex] ^=
            (ctx.saved ^ ((uint64_t) ((uint64_t) (0x02 | (1 << 2)) <<
                            ((ctx.byteIndex) * 8))));
#else
    /* For testing the "pure" Keccak version */
    ctx.s[ctx.wordIndex] ^=
            (ctx.saved ^ ((uint64_t) ((uint64_t) 1 << (ctx.byteIndex *
                                    8))));
#endif

    ctx.s[SHA3_KECCAK_SPONGE_WORDS - ctx.capacityWords - 1] ^=
            SHA3_CONST(0x8000000000000000UL);
    keccakf();

    /* Return first bytes of the ctx.s. This conversion is not needed for
     * little-endian platforms e.g. wrap with #if !defined(__BYTE_ORDER__)
     * || !defined(__ORDER_LITTLE_ENDIAN__) || \
     * __BYTE_ORDER__!=__ORDER_LITTLE_ENDIAN__ ... the conversion below ...
     * #endif */
    {
        for(word = 0; word < SHA3_KECCAK_SPONGE_WORDS; word++) {
            t1 = (uint32_t) ctx.s[word];
            t2 = (uint32_t) ((ctx.s[word] >> 16) >> 16);
            ctx.sb[word * 8 + 0] = (uint8_t) (t1);
            ctx.sb[word * 8 + 1] = (uint8_t) (t1 >> 8);
            ctx.sb[word * 8 + 2] = (uint8_t) (t1 >> 16);
            ctx.sb[word * 8 + 3] = (uint8_t) (t1 >> 24);
            ctx.sb[word * 8 + 4] = (uint8_t) (t2);
            ctx.sb[word * 8 + 5] = (uint8_t) (t2 >> 8);
            ctx.sb[word * 8 + 6] = (uint8_t) (t2 >> 16);
            ctx.sb[word * 8 + 7] = (uint8_t) (t2 >> 24);
        }
    }

    SHA3_TRACE_BUF("Hash: (first 32 bytes)", ctx.sb, 256 / 8);

    // return (ctx.sb);
}
