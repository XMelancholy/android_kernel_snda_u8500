
/*
 * Copyright (C) ST-Ericsson SA 2012
 *
 * License Terms: GNU General Public License v2
 * Author: Pankaj Chikhale <pankaj.chikhale@stericsson.com>
 *
 * SHRM tracing
 */

#if !defined(_TRACE_SHRM_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_SHRM_H

#include <linux/types.h>
#include <linux/tracepoint.h>

#undef TRACE_SYSTEM
#define TRACE_SYSTEM shrm
#define TRACE_INCLUDE_FILE shrm-trace

DECLARE_EVENT_CLASS(shrm_no_arg,

	TP_PROTO(u8 dummy),

	TP_ARGS(dummy),

	TP_STRUCT__entry(
		__field(u8, dummy)
	),

	TP_fast_assign(
		__entry->dummy = dummy;
	),

	TP_printk("%d", __entry->dummy)
);

DEFINE_EVENT(shrm_no_arg, shrm_ca_sleep_req,

	TP_PROTO(u8 dummy),

	TP_ARGS(dummy)
);

DEFINE_EVENT(shrm_no_arg, shrm_ca_wake_req,

	TP_PROTO(u8 dummy),

	TP_ARGS(dummy)
);

DEFINE_EVENT(shrm_no_arg, shrm_send_ac_msg_pend_notify_0,

	TP_PROTO(u8 dummy),

	TP_ARGS(dummy)
);

DEFINE_EVENT(shrm_no_arg, shrm_send_ac_msg_pend_notify_1,

	TP_PROTO(u8 dummy),

	TP_ARGS(dummy)
);

DEFINE_EVENT(shrm_no_arg, shrm_wake_irq_handler,

	TP_PROTO(u8 dummy),

	TP_ARGS(dummy)
);

DEFINE_EVENT(shrm_no_arg, shrm_ca_msg_pending_notify_0_irq_handler,

	TP_PROTO(u8 dummy),

	TP_ARGS(dummy)
);

DEFINE_EVENT(shrm_no_arg, shrm_ca_msg_pending_notify_1_irq_handler,

	TP_PROTO(u8 dummy),

	TP_ARGS(dummy)
);

DEFINE_EVENT(shrm_no_arg, shrm_ca_msg_read_notification_0,

	TP_PROTO(u8 dummy),

	TP_ARGS(dummy)
);

DEFINE_EVENT(shrm_no_arg, shrm_ca_msg_read_notification_1,

	TP_PROTO(u8 dummy),

	TP_ARGS(dummy)
);


#endif /* _TRACE_SHRM_H */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH ../../drivers/modem/shrm/

/* This part must be outside protection */
#include <trace/define_trace.h>

