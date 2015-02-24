#undef TRACE_SYSTEM
#define TRACE_SYSTEM synaptics_trace

#if !defined(_TRACE_SYNAPTICS_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_SYNAPTICS_TRACE_H

#include <linux/types.h>
#include <linux/tracepoint.h>
#include <trace/events/gfpflags.h>

TRACE_EVENT(tracing_mark_write,

	TP_PROTO(pid_t pid, char *message, int evtype),

	TP_ARGS(pid, message, evtype),

	TP_STRUCT__entry(
		__field(        pid_t,		pid 		)
		__field(	char *,		message	)
		__field(	int,		evtype	)
	),

	TP_fast_assign(
		__entry->pid 		= pid;
		__entry->message	= message;
		__entry->evtype   = evtype;
	),

	TP_printk("C|%d|%s|%d",
		__entry->pid,
		__entry->message,
		__entry->evtype)

);

#endif /* _TRACE_TOUCH_H */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
/* This part must be outside protection */
#include <trace/define_trace.h>
