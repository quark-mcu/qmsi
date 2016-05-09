/*
 *{%copyright}
 */

#ifndef __QFU_PACKETS_H__
#define __QFU_PACKETS_H__

#include <stdint.h>

/*
 * NOTE: different compilers use different directives for packing structs;
 * therefore we define a macro whose value will depend on the specific compiler
 * used.
 *
 * For now, we set it to the GCC directive.
 */
#define __QM_ATTR_PACKED__ __attribute__((__packed__))

/* QFU_HDR_MAGIC = "QFUH" */
#define QFU_HDR_MAGIC (0x48554651)
/**
 * The enumeration of possible authentication mechanism.
 */
typedef enum {
	QFU_AUTH_NONE = 0, /**< No authentication. */
} qfu_auth_type_t;

/**
 * The structure of the QFU header.
 */
typedef struct __QM_ATTR_PACKED__ {
	uint32_t magic;     /**< Header magic: 'QFUH'. */
	uint16_t vid;       /**< Target Vendor ID. */
	uint16_t pid;       /**< Target Product ID. */
	uint16_t pid_dfu;   /**< Target Product ID when in DFU mode. */
	uint16_t partition; /**< Target partition ID. */
	uint32_t version;   /**< Firmware version. */
	uint16_t block_sz;  /**< Block size. */
	uint16_t n_blocks;  /**< Total number of blocks; incl. the header. */
	uint16_t cipher;    /**< Kind of authentication (cipher suite) used. */
} qfu_hdr_t;

#endif /* __QFU_PACKETS_H__ */
