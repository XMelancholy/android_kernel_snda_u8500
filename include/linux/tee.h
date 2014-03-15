/*
 * Trusted Execution Environment (TEE) interface for TrustZone enabled ARM
 * CPUs.
 *
 * Copyright (C) ST-Ericsson SA 2010
 * Author: Shujuan Chen <shujuan.chen@stericsson.com>
 * Author: Martin Hovang <martin.xm.hovang@stericsson.com>
 * Author: Joakim Bech <joakim.xx.bech@stericsson.com>
 * License terms: GNU General Public License (GPL) version 2
 */

#ifndef TEE_H
#define TEE_H

#define TEE_VERSION_017 0
#define TEE_VERSION_100 1

#if defined(__KERNEL__)
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#else
#include <stdint.h>
typedef uint32_t __u32;
typedef uint16_t __u16;
typedef uint8_t __u8;
typedef int32_t s32;
#endif

/* tee_cmd id values */
#define TEED_OPEN_SESSION    0x00000000U
#define TEED_CLOSE_SESSION   0x00000001U
#define TEED_INVOKE          0x00000002U

/* tee_retval id values */
#define TEED_SUCCESS                0x00000000U
#define TEED_ERROR_GENERIC          0xFFFF0000U
#define TEED_ERROR_ACCESS_DENIED    0xFFFF0001U
#define TEED_ERROR_CANCEL           0xFFFF0002U
#define TEED_ERROR_ACCESS_CONFLICT  0xFFFF0003U
#define TEED_ERROR_EXCESS_DATA      0xFFFF0004U
#define TEED_ERROR_BAD_FORMAT       0xFFFF0005U
#define TEED_ERROR_BAD_PARAMETERS   0xFFFF0006U
#define TEED_ERROR_BAD_STATE        0xFFFF0007U
#define TEED_ERROR_ITEM_NOT_FOUND   0xFFFF0008U
#define TEED_ERROR_NOT_IMPLEMENTED  0xFFFF0009U
#define TEED_ERROR_NOT_SUPPORTED    0xFFFF000AU
#define TEED_ERROR_NO_DATA          0xFFFF000BU
#define TEED_ERROR_OUT_OF_MEMORY    0xFFFF000CU
#define TEED_ERROR_BUSY             0xFFFF000DU
#define TEED_ERROR_COMMUNICATION    0xFFFF000EU
#define TEED_ERROR_SECURITY         0xFFFF000FU
#define TEED_ERROR_SHORT_BUFFER     0xFFFF0010U

/* TEE origin codes */
#define TEED_ORIGIN_DRIVER          0x00000002U
#define TEED_ORIGIN_TEE             0x00000003U
#define TEED_ORIGIN_TEE_APPLICATION 0x00000004U

#define TEE_UUID_CLOCK_SIZE 8

#define TEEC_CONFIG_PAYLOAD_REF_COUNT 4

/*
 * Flag constants indicating which of the memory references in an open session
 * or invoke command operation payload (TEEC_Operation) that are used.
 */
#define TEEC_MEMREF_0_USED  0x00000001
#define TEEC_MEMREF_1_USED  0x00000002
#define TEEC_MEMREF_2_USED  0x00000004
#define TEEC_MEMREF_3_USED  0x00000008

/*
 * Flag constants indicating the type of parameters encoded inside the
 * operation payload (TEEC_Operation), Type is uint32_t.
 *
 * TEEC_NONE         The Parameter is not used
 * TEEC_VALUE_INPUT  The Parameter is a TEEC_Value tagged as input.
 * TEEC_VALUE_OUTPUT The Parameter is a TEEC_Value tagged as output.
 * TEEC_VALUE_INOUT  The Parameter is a TEEC_Value tagged as both as input and
 *                   output, i.e., for which both the behaviors of
 *                   TEEC_VALUE_INPUT and TEEC_VALUE_OUTPUT apply.
 * TEEC_MEMREF_TEMP_INPUT  The Parameter is a TEEC_TempMemoryReference
 *                         describing a region of memory which needs to be
 *                         temporarily registered for the duration of the
 *                         Operation and is tagged as input.
 * TEEC_MEMREF_TEMP_OUTPUT Same as TEEC_MEMREF_TEMP_INPUT, but the Memory
 *                         Reference is tagged as output. The Implementation
 *                         may update the size field to reflect the required
 *                         output size in some use cases.
 * TEEC_MEMREF_TEMP_INOUT  A Temporary Memory Reference tagged as both input
 *                         and output, i.e., for which both the behaviors of
 *                         TEEC_MEMREF_TEMP_INPUT and TEEC_MEMREF_TEMP_OUTPUT
 *                         apply.
 * TEEC_MEMREF_WHOLE  The Parameter is a Registered Memory Reference that
 *                    refers to the entirety of its parent Shared Memory block.
 *                    The parameter structure is a TEEC_MemoryReference. In
 *                    this structure, the Implementation MUST read only the
 *                    parent field and MAY update the size field when the
 *                    operation completes.
 * TEEC_MEMREF_PARTIAL_INPUT A Registered Memory Reference structure that
 *                           refers to a partial region of its parent Shared
 *                           Memory block and is tagged as input.
 * TEEC_MEMREF_PARTIAL_OUTPUT  Registered Memory Reference structure that
 *                             refers to a partial region of its parent Shared
 *                             Memory block and is tagged as output.
 * TEEC_MEMREF_PARTIAL_INOUT  The Registered Memory Reference structure that
 *                            refers to a partial region of its parent Shared
 *                            Memory block and is tagged as both input and
 *                            output, i.e., for which both the behaviors of
 *                            TEEC_MEMREF_PARTIAL_INPUT and
 *                            TEEC_MEMREF_PARTIAL_OUTPUT apply.
 */
#define TEEC_NONE                   0x00000000
#define TEEC_VALUE_INPUT            0x00000001
#define TEEC_VALUE_OUTPUT           0x00000002
#define TEEC_VALUE_INOUT            0x00000003
#define TEEC_MEMREF_TEMP_INPUT      0x00000005
#define TEEC_MEMREF_TEMP_OUTPUT     0x00000006
#define TEEC_MEMREF_TEMP_INOUT      0x00000007
#define TEEC_MEMREF_WHOLE           0x0000000C
#define TEEC_MEMREF_PARTIAL_INPUT   0x0000000D
#define TEEC_MEMREF_PARTIAL_OUTPUT  0x0000000E
#define TEEC_MEMREF_PARTIAL_INOUT   0x0000000F

/*
 * Flag constants indicating the data transfer direction of memory in
 * TEEC_SharedMemory and TEEC_MemoryReference. TEEC_MEM_INPUT signifies data
 * transfer direction from the client application to the TEE. TEEC_MEM_OUTPUT
 * signifies data transfer direction from the TEE to the client application.
 */
#define TEEC_MEM_INPUT  0x00000001
#define TEEC_MEM_OUTPUT 0x00000002

/*
 * Session login methods, for use in TEEC_OpenSession() as parameter
 * connectionMethod. Type is t_uint32.
 *
 * TEEC_LOGIN_PUBLIC  No login data is provided.
 */
#define TEEC_LOGIN_PUBLIC 0x0
#define TEEC_LOGIN_KERNEL 0xa0000000

/*
 * Exposed functions (command_id) in the static TA
 */
#define TEE_STA_GET_PRODUCT_CONFIG		10
#define TEE_STA_SET_L2CC_PREFETCH_CTRL_REGISTER 11
#define TEE_STA_CONF_SHARED_MUTEX		34
#define TEE_STA_OPEN_SHARED_MEMORY 39

/* Flags indicating run-time environment */
#define TEE_RT_FLAGS_NORMAL		0x00000000
#define TEE_RT_FLAGS_MASK_ITP_PROD      0x00000001
#define TEE_RT_FLAGS_MODEM_DEBUG	0x00000002
#define TEE_RT_FLAGS_RNG_REG_PUBLIC     0x00000004
#define TEE_RT_FLAGS_JTAG_ENABLED       0x00000008

/*
 * Product id numbers
 */
#define TEE_PRODUCT_ID_UNKNOWN    0
#define TEE_PRODUCT_ID_8400       1 /* Obsolete */
#define TEE_PRODUCT_ID_8500B      2 /* 1080p/1GHz/400MHz */
#define TEE_PRODUCT_ID_9500       3 /* 1080p/1GHz/400MHz */
#define TEE_PRODUCT_ID_5500       4 /* Obsolete */
#define TEE_PRODUCT_ID_7400       5
#define TEE_PRODUCT_ID_8500C      6 /* 720p/1GHz/400MHz */
#define TEE_PRODUCT_ID_8500A      7 /* 720p/800MHz/320MHz */
#define TEE_PRODUCT_ID_8500E      8 /* 1080p/1.15GHz/533MHz */
#define TEE_PRODUCT_ID_8520F      9 /* 720p/1.15GHz/533MHz */
#define TEE_PRODUCT_ID_8520H     10 /* 720p/1GHz/200MHz */
#define TEE_PRODUCT_ID_9540      11
#define TEE_PRODUCT_ID_9500C     12 /* 1080p/1.15GHz/533MHz */
#define TEE_PRODUCT_ID_8500F     13
#define TEE_PRODUCT_ID_8540APE   14
#define TEE_PRODUCT_ID_8540XMIP  15
#define TEE_PRODUCT_ID_8520E     16
#define TEE_PRODUCT_ID_8520J     17

/* Flags indicating fuses */
#define TEE_FUSE_FLAGS_MODEM_DISABLE    0x00000001

/**
 * struct tee_uuid - Structure that represent an uuid.
 * @timeLow: The low field of the time stamp.
 * @timeMid: The middle field of the time stamp.
 * @timeHiAndVersion: The high field of the timestamp multiplexed
 *                    with the version number.
 * @clockSeqAndNode: The clock sequence and the node.
 *
 * This structure have different naming (camel case) to comply with Global
 * Platforms TEE Client API spec. This type is defined in RFC4122.
 */
struct tee_uuid {
	__u32 timeLow;
	__u16 timeMid;
	__u16 timeHiAndVersion;
	__u8 clockSeqAndNode[TEE_UUID_CLOCK_SIZE];
};

/**
 * struct tee_identity - Represents the identity of the client
 * @login: Login id
 * @uuid: UUID as defined above
 */
struct tee_identity {
	__u32 login;
	struct tee_uuid uuid;
};

/**
 * struct TEEC_TempMemoryReference - Temporary memory to transfer data between
 * a client application and trusted code, only used for the duration of the
 * operation.
 * @buffer:  The memory buffer which is to be, or has been shared with the TEE.
 * @size: The size, in bytes, of the memory buffer.
 *
 * A memory buffer that is registered temporarily for the duration of the
 * operation to be called.
 */
struct tee_tmpref {
	void *buffer;
	size_t size;
};

/**
 * struct tee_sharedmemory - Shared memory block for TEE.
 * @buffer: The in/out data to TEE.
 * @size: The size of the data.
 * @flags: Variable telling whether it is a in, out or in/out parameter.
 * @hwmem_fd: !Internal! HWMEM file descriptor
 * @hwmem_lid: !Internal! HWMEM local id
 * @hwmem_gname: !Internal! HWMEM global name
 */
struct tee_sharedmemory {
	void *buffer;
	size_t size;
	__u32 flags;
	s32 hwmem_fd;
	s32 hwmem_lid;
	s32 hwmem_gname;
};

/**
 * struct TEEC_RegisteredMemoryReference - use a pre-registered or
 * pre-allocated shared memory block of memory to transfer data between
 * a client application and trusted code.
 * @parent: Points to a shared memory structure. The memory reference
 *          may utilize the whole shared memory or only a part of it.
 *          Must not be NULL
 * @size: The size, in bytes, of the memory buffer.
 * @offset: The offset, in bytes, of the referenced memory region from
 *          the start of the shared memory block.
 */
struct tee_regref {
	struct tee_sharedmemory *parent;
	size_t size;
	size_t offset;
};

/**
 * struct TEEC_Value - Small raw data container
 * @a:  The first integer value.
 * @b:  The second second value.
 *
 * Instead of allocating a shared memory buffer this structure can be used
 * to pass small raw data between a client application and trusted code.
 */
struct tee_value {
	__u32 a;
	__u32 b;
};

/**
 * union tee_parameter - Memory container to be used when passing data between
 *                       client application and trusted code.
 * @tmpref: A temporary memory reference only valid for the duration
 *          of the operation.
 * @memref: The entire shared memory or parts of it.
 * @value: The small raw data container to use
 *
 * Either the client uses a shared memory reference, parts of it or a small raw
 * data container.
 */
union tee_parameter {
	struct tee_tmpref tmpref;
	struct tee_regref memref;
	struct tee_value  value;
};

/**
 * struct tee_operation - Payload for sessions or invoke operation.
 * @shm: Array containing the shared memory buffers.
 * @flags: Tells which if memory buffers that are in use.
 */
struct tee_operation {
	__u32 started;
	__u32 types;
	union tee_parameter param[TEEC_CONFIG_PAYLOAD_REF_COUNT];
	struct tee_sharedmemory shm[TEEC_CONFIG_PAYLOAD_REF_COUNT];
	__u32 flags;
	__u32 param_flags[TEEC_CONFIG_PAYLOAD_REF_COUNT];
};

struct tee_context {};

/**
 * struct tee_cmd - The data struct used in user space communication.
 * @driver_cmd: The command type to be executed by the driver. This is used
 *              from a client (user space) to tell the Linux kernel whether
 *              it's a open session, close session or if it is an invoke
 *              command.
 * @err: Error code (as in Global Platform TEE Client API spec)
 * @origin: Origin for the error code (also from GP spec).
 * @cmd: The command ID to be executed by the trusted application.
 * @uuid: The uuid for the trusted application.
 * @data: Pointer to the cmd data as a trusted application binary.
 * @ta_size: The size of the cmd data.
 * @op: The operation payload for the trusted application function.
 *
 * This structure is used by the Linux kernel to communicate with the user
 * space.
 */
struct tee_cmd {
	__u32 driver_cmd;
	__u32 err;
	__u32 origin;
	__u32 cmd;
	struct tee_uuid *uuid;
	void *data;
	__u32 data_size;
	struct tee_operation *op;
};

/**
 * struct tee_session - The session data of an open tee device.
 * @uc: The command struct
 * @id: The session ID returned and managed by the secure world
 * @state: The current state of the session in the Linux kernel.
 * @vaddr: Virtual address for the operation memrefs currently in use.
 */
struct tee_session {
	struct tee_cmd uc;
	__u32 id;
	__u32 state;
	__u32 login;
	__u32 *vaddr[TEEC_CONFIG_PAYLOAD_REF_COUNT];
};

/**
 * struct tee_read - Contains the error message and the origin.
 * @err: Error code (as in Global Platform TEE Client API spec)
 * @origin: Origin for the error code (also from spec).
 *
 * This is used by user space when a user space application wants to get more
 * information about an error.
 */
struct tee_read {
	unsigned int err; /* return value */
	unsigned int origin; /* error origin */
};

#define TEE_RESOLVE_IOC _IOWR('t', 160, struct tee_sharedmemory)

/**
 * struct ta_addr - Struct that acts as a helper struct when it comes to
 * allocating physically contiguous memory.
 *
 * @paddr: Represents the physical address of a buffer.
 * @vaddr: Represents the virtual address of a buffer.
 * @alloc: A pointer to the hwmem allocation structure.
 */
struct ta_addr {
	void *paddr;
	void *vaddr;
	struct hwmem_alloc *alloc;
};

/*
 * TEEC_PARAM_TYPES() - Encode the paramTypes according to the supplied types.
 * @param p0: The first param type
 * @param p1: The second param type
 * @param p2: The third param type
 * @param p3: The fourth param type
 */
#define TEEC_PARAM_TYPES(p0, p1, p2, p3) \
	((p0) | ((p1) << 4) | ((p2) << 8) | ((p3) << 12))

/**
 * Get the i_th param type from the paramType
 * @param p: The paramType
 * @param i: The i-th parameter to get the type for
 */
#define TEEC_PARAM_TYPE_GET(p, i) (((p) >> (i * 4)) & 0xF)

#if defined(__KERNEL__)
/**
 * struct tee_product_config - System configuration structure.
 *
 * @product_id: Product identification.
 * @rt_flags: Runtime configuration flags.
 * @fuse_flags: Fuse flags.
 *
 */
struct tee_product_config {
	__u32 product_id;
	__u32 rt_flags;
	__u32 fuse_flags;
};

/**
 * struct tee_export_operations - Functions that are exported for modules within
 * the kernel.
 *
 * @initialize: The function that initialize a TEE context.
 * @finalize: The function that finalize a TEE context.
 * @operation: The function that open the session towards a TEE application.
 * @close_session: The function that close the session towards a TEE
 *                 application.
 * @invoke: The function that makes the call to a function withing a TEE
 *          application.
 * @allocate_shared_memory: The function that allocates shared memory.
 * @release_shared_memory: The function that releases shared memory.
 */
struct tee_export_operations {
	int (*initialize)(const char *name, struct tee_context *context);
	int (*finalize)(struct tee_context *context);
	int (*open_session)(struct tee_context *context,
		    struct tee_session *session,
		    const struct tee_uuid *destination,
		    unsigned int connection_method,
		    void *connection_data, struct tee_operation *operation,
		    unsigned int *error_origin);
	int (*close_session)(struct tee_session *session);
	int (*invoke)(struct tee_session *session,
		      unsigned int command_id,
		      struct tee_operation *operation,
		      unsigned int *error_origin);
	int (*allocate_shared_memory)(struct tee_context *context,
				      struct tee_sharedmemory *shared_memory);
	void (*release_shared_memory)(struct tee_sharedmemory *shared_memory);
};

/**
 * struct tee_internal_operations - Functions that needs to be a part of the tee
 * implementation in the driver.
 *
 * @init: Platform specific init function that is called when the driver is
 *        probed.
 * @exit: Platform specific exit function that is called when the driver is
 *        unloaded.
 * @setup: The setup function for the driver.
 */
struct tee_internal_operations {
	int (*init)(struct platform_device *pd);
	int (*exit)(void);
	int (*setup)(char *str);
};

/**
 * struct tee_platform_data - device specific data.
 *
 * Platform data used to get back in the probe function when the driver has been
 * loaded.
 *
 * @api_version: The version of the Global platform specification.
 ' @bridge_func_addr: The address of the function making the bridge towards the
 * secure world.
 * @eops: Functions exported in the kernel.
 * @fops: File operation for the character device.
 * @tops: API specfic init, exit and setup function.
 * @misc_dev: The device used to communicate with tee driver.
 */
struct tee_platform_data {
	const __u32 api_version;
	const __u32 bridge_func_addr;
	struct tee_export_operations eops;
	struct file_operations fops;
	struct tee_internal_operations tops;
	struct miscdevice misc_dev;
};

/**
 * Function used to query whether the probing of tee driver is ready or not.
 */
bool is_teec_ready(void);


/**
 * Driver init function for the Global Platform API v0.17 version.
 *
 * @param pd: The platform device used for the tee driver.
 */
int tee_init_017(struct platform_device *pd);

/**
 * Driver init function for the Global Platform API v1.00 version.
 *
 * @param pd: The platform device used for the tee driver.
 */
int tee_init_100(struct platform_device *pd);
#endif


/**
 * tee_mem_init() - Function to register tee memory
 * @param addr: physical addr of memory
 * @param size: size of memory in bytes
 */
int tee_mem_init(__u32 addr, __u32 size);

/**
 * tee_register_rpc() - Function to register the Remote Procedure Call (RPC)
 * from secure world
 * @param fnk: RPC function to register
 * @param bf: RPC buffer
 * @param nbr_bf: Number of RPC buffers
 */
int tee_register_rpc(void *fnk, void* bf, __u32 nbr_bf);

/**
 * teec_initialize_context() - Initializes a context holding connection
 * information on the specific TEE.
 * @param name: A zero-terminated string identifying the TEE to connect to.
 *              If name is set to NULL, the default TEE is connected to.
 *              NULL is the only supported value in this version of the
 *              API implementation.
 * @param context: The context structure which is to be initialized.
 *
 * Initializes a context holding connection information between the calling
 * client application and the TEE designated by the name string.
 */
int teec_initialize_context(const char *name, struct tee_context *context);

/**
 * teec_finalize_context() - Destroys a context holding connection information
 * on the specific TEE.
 * @param context: The context to be destroyed.
 *
 * This function destroys an initialized TEE context, closing the connection
 * between the client application and the TEE. This function must only be
 * called when all sessions related to this TEE context have been closed and
 * all shared memory blocks have been released.
 */
int teec_finalize_context(struct tee_context *context);

/**
 * teec_open_session() - Opens a new session with the specified trusted
 * application.
 * @param context: The initialized TEE context structure in which scope to
 *                 open the session.
 * @param session: The session to initialize.
 * @param destination: A structure identifying the trusted application with
 *                     which to open a session. If this is set to NULL the
 *                     operation TEEC_MEMREF_0 is expected to contain the blob
 *                     which holds the Trusted Application.
 * @param connection_method: The connection method to use.
 * @param connection_data: Any data necessary to connect with the chosen
 *                         connection method. Not supported should be set to
 *                         NULL.
 * @param operation: An operation structure to use in the session. May be
 *                   set to NULL to signify no operation structure needed.
 *                   If destination is set to NULL, TEEC_MEMREF_0 is
 *                   expected to hold the TA binary as described above.
 * @param error_origin: A parameter which will hold the error origin if this
 *                      function returns any value other than TEEC_SUCCESS.
 *
 * Opens a new session with the specified trusted application. Only
 * connectionMethod == TEEC_LOGIN_PUBLIC is supported. connectionData and
 * operation shall be set to NULL.
 */
int teec_open_session(struct tee_context *context, struct tee_session *session,
		      const struct tee_uuid *destination,
		      unsigned int connection_method,
		      void *connection_data, struct tee_operation *operation,
		      unsigned int *error_origin);

/**
 * teec_close_session() - Closes the session which has been opened with the
 * specific trusted application.
 * @param session: The opened session to close.
 *
 * Closes the session which has been opened with the specific trusted
 * application.
 */
int teec_close_session(struct tee_session *session);

/**
 * teec_invoke_command() - Executes a command in the specified trusted
 * application.
 * @param destination: A structure identifying the trusted application.
 * @param command_id: Identifier of the command in the trusted application to
 *                    invoke.
 * @param operation: An operation structure to use in the invoke command. May
 *                   be set to NULL to signify no operation structure needed.
 * @param error_origin: A parameter which will hold the error origin if this
 *                      function returns any value other than TEEC_SUCCESS.
 *
 * Executes a command in the specified trusted application.
 */
int teec_invoke_command(struct tee_session *session, unsigned int command_id,
			struct tee_operation *operation,
			unsigned int *error_origin);

/**
 * teec_allocate_shared_memory() - Allocate shared memory for TEE.
 * @param context: The initialized TEE context structure in which scope to
 *                 open the session.
 * @param shared_memory: Pointer to the allocated shared memory.
 */
int teec_allocate_shared_memory(struct tee_context *context,
				struct tee_sharedmemory *shared_memory);

/**
 * teec_release_shared_memory() - Free the shared memory.
 * @param shared_memory: Pointer to the shared memory to be freed.
 */
void teec_release_shared_memory(struct tee_sharedmemory *shared_memory);

#endif
