/*
 *{%copyright}
 */

#ifndef __QDM_PACKETS_H__
#define __QDM_PACKETS_H__

#include <stdint.h>

/*
 * NOTE: different compilers use different directives for packing structs;
 * therefore we define a macro whose value will depend on the specific compiler
 * used.
 * For now, we set it to the GCC directive.
 */
#define __QM_ATTR_PACKED__ __attribute__((__packed__))

/**
 * The enumeration of QDM packet types.
 */
typedef enum {
	/* Requests */
	QDM_SYS_INFO_REQ = 0x444D0000, /**< System Information request. */
	QDM_APP_ERASE = 0x444D0001,    /**< Application Erase request. */
	QDM_UPDATE_KEY = 0x444D0002,   /**< Crypto Key Update request. */
	/* Responses */
	QDM_SYS_INFO_RSP = 0x444D8000, /**< System Information response. */
} qdm_pkt_type_t;

/**
 * The generic structure of a QDM packet.
 */
typedef struct __QM_ATTR_PACKED__ {
	uint32_t type;     /**< The packet type, see qdm_pkt_type_t enum. */
	uint8_t payload[]; /**< The type-specific payload/structure. */
} qdm_generic_pkt_t;

/**
 * Type-specific structure for the QDM System Information response packet.
 */
/*
 * NOTE: this is a first stub to be completed with other fields when
 * defined.
 */
typedef struct __QM_ATTR_PACKED__ {
	uint16_t sysupd_version; /**< The bootloader version. */
	uint8_t soc_type;	/**< The SOC type. */
	uint8_t auth_type;       /**< The active authentication type. */
} qdm_sys_info_rsp_payload_t;

#endif /* __QDM_PACKETS_H__ */
