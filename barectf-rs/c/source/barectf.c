/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2015-2020 Philippe Proulx <pproulx@efficios.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 *
 * The following code was generated by barectf v3.1.0-dev
 * on 2022-03-22T05:06:41.968657.
 *
 * For more details, see <https://barectf.org/>.
 */

#include <stdint.h>
#include <string.h>
#include <assert.h>

#include "barectf.h"
#include "barectf-bitfield.h"

#define _ALIGN(_at_var, _align)						\
	do {								\
		(_at_var) = ((_at_var) + ((_align) - 1)) & -(_align);	\
	} while (0)

#ifdef __cplusplus
# define _TO_VOID_PTR(_value)		static_cast<void *>(_value)
# define _FROM_VOID_PTR(_type, _value)	static_cast<_type *>(_value)
#else
# define _TO_VOID_PTR(_value)		((void *) (_value))
# define _FROM_VOID_PTR(_type, _value)	((_type *) (_value))
#endif

#define _BITS_TO_BYTES(_x)	((_x) >> 3)
#define _BYTES_TO_BITS(_x)	((_x) << 3)

union _f2u {
	float f;
	uint32_t u;
};

union _d2u {
	double f;
	uint64_t u;
};

uint32_t barectf_packet_size(const void * const vctx)
{
	return _FROM_VOID_PTR(const struct barectf_ctx, vctx)->packet_size;
}

int barectf_packet_is_full(const void * const vctx)
{
	const struct barectf_ctx * const ctx = _FROM_VOID_PTR(const struct barectf_ctx, vctx);

	return ctx->at == ctx->packet_size;
}

int barectf_packet_is_empty(const void * const vctx)
{
	const struct barectf_ctx * const ctx = _FROM_VOID_PTR(const struct barectf_ctx, vctx);

	return ctx->at <= ctx->off_content;
}

uint32_t barectf_packet_events_discarded(const void * const vctx)
{
	return _FROM_VOID_PTR(const struct barectf_ctx, vctx)->events_discarded;
}

uint32_t barectf_discarded_event_records_count(const void * const vctx)
{
	return barectf_packet_events_discarded(vctx);
}

uint32_t barectf_packet_sequence_number(const void * const vctx)
{
	return _FROM_VOID_PTR(const struct barectf_ctx, vctx)->sequence_number;
}

uint8_t *barectf_packet_buf(const void * const vctx)
{
	return _FROM_VOID_PTR(const struct barectf_ctx, vctx)->buf;
}

uint8_t *barectf_packet_buf_addr(const void * const vctx)
{
	return barectf_packet_buf(vctx);
}

uint32_t barectf_packet_buf_size(const void * const vctx)
{
	const struct barectf_ctx * const ctx = _FROM_VOID_PTR(const struct barectf_ctx, vctx);

	return _BITS_TO_BYTES(ctx->packet_size);
}

void barectf_packet_set_buf(void * const vctx, uint8_t * const buf,
		const uint32_t buf_size)
{
	struct barectf_ctx * const ctx = _FROM_VOID_PTR(struct barectf_ctx, vctx);

	ctx->buf = buf;

	if (ctx->at == ctx->packet_size) {
		/* Keep full packet state */
		ctx->at = _BYTES_TO_BITS(buf_size);
	}

	ctx->packet_size = _BYTES_TO_BITS(buf_size);
}

int barectf_packet_is_open(const void * const vctx)
{
	return _FROM_VOID_PTR(const struct barectf_ctx, vctx)->packet_is_open;
}

int barectf_is_in_tracing_section(const void * const vctx)
{
	return _FROM_VOID_PTR(const struct barectf_ctx, vctx)->in_tracing_section;
}

volatile const int *barectf_is_in_tracing_section_ptr(const void * const vctx)
{
	return &_FROM_VOID_PTR(const struct barectf_ctx, vctx)->in_tracing_section;
}

int barectf_is_tracing_enabled(const void * const vctx)
{
	return _FROM_VOID_PTR(const struct barectf_ctx, vctx)->is_tracing_enabled;
}

void barectf_enable_tracing(void * const vctx, const int enable)
{
	_FROM_VOID_PTR(struct barectf_ctx, vctx)->is_tracing_enabled = enable;
}

static
void _write_c_str(struct barectf_ctx * const ctx, const char * const src)
{
	const uint32_t sz = strlen(src) + 1;

	memcpy(&ctx->buf[_BITS_TO_BYTES(ctx->at)], src, sz);
	ctx->at += _BYTES_TO_BITS(sz);
}

static
int _reserve_er_space(void * const vctx, const uint32_t er_size)
{
	int ret;
	struct barectf_ctx * const ctx = _FROM_VOID_PTR(struct barectf_ctx, vctx);

	/* Event _cannot_ fit? */
	if (er_size > (ctx->packet_size - ctx->off_content)) {
		goto no_space;
	}

	/* Packet is full? */
	if (barectf_packet_is_full(ctx)) {
		/* Yes: is the back end full? */
		if (ctx->cbs.is_backend_full(ctx->data)) {
			/* Yes: discard event record */
			goto no_space;
		}

		/* Back-end is _not_ full: open new packet */
		ctx->use_cur_last_event_ts = 1;
		ctx->cbs.open_packet(ctx->data);
		ctx->use_cur_last_event_ts = 0;
	}

	/* Event fits the current packet? */
	if (er_size > (ctx->packet_size - ctx->at)) {
		/* No: close packet now */
		ctx->use_cur_last_event_ts = 1;
		ctx->cbs.close_packet(ctx->data);
		ctx->use_cur_last_event_ts = 0;

		/* Is the back end full? */
		if (ctx->cbs.is_backend_full(ctx->data)) {
			/* Yes: discard event record */
			goto no_space;
		}

		/* Back-end is _not_ full: open new packet */
		ctx->use_cur_last_event_ts = 1;
		ctx->cbs.open_packet(ctx->data);
		ctx->use_cur_last_event_ts = 0;
		assert(er_size <= (ctx->packet_size - ctx->at));
	}

	ret = 1;
	goto end;

no_space:
	ctx->events_discarded++;
	ret = 0;

end:
	return ret;
}

static
void _commit_er(void * const vctx)
{
	struct barectf_ctx * const ctx = _FROM_VOID_PTR(struct barectf_ctx, vctx);

	/* Is the packet full? */
	if (barectf_packet_is_full(ctx)) {
		/* Yes: close it now */
		ctx->cbs.close_packet(ctx->data);
	}
}

/* Initialize context */
void barectf_init(void *vctx,
	uint8_t * const buf, const uint32_t buf_size,
	const struct barectf_platform_callbacks cbs, void * const data)
{
	struct barectf_ctx * const ctx = _FROM_VOID_PTR(struct barectf_ctx, vctx);
	ctx->cbs = cbs;
	ctx->data = data;
	ctx->buf = buf;
	ctx->packet_size = _BYTES_TO_BITS(buf_size);
	ctx->at = 0;
	ctx->events_discarded = 0;
	ctx->sequence_number = 0;
	ctx->packet_is_open = 0;
	ctx->in_tracing_section = 0;
	ctx->is_tracing_enabled = 1;
	ctx->use_cur_last_event_ts = 0;
}

/* Open packet for data stream type `default` */
void barectf_default_open_packet(
	struct barectf_default_ctx * const sctx)
{
	struct barectf_ctx * const ctx = &sctx->parent;
	const uint32_t ts = ctx->use_cur_last_event_ts ?
		sctx->cur_last_event_ts :
		ctx->cbs.default_clock_get_value(ctx->data);
	const int saved_in_tracing_section = ctx->in_tracing_section;

	/*
	 * This function is either called by a tracing function, or
	 * directly by the platform.
	 *
	 * If it's called by a tracing function, then
	 * `ctx->in_tracing_section` is 1, so it's safe to open
	 * the packet here (alter the packet), even if tracing was
	 * disabled in the meantime because we're already in a tracing
	 * section (which finishes at the end of the tracing function
	 * call).
	 *
	 * If it's called directly by the platform, then if tracing is
	 * disabled, we don't want to alter the packet, and return
	 * immediately.
	 */
	if (!ctx->is_tracing_enabled && !saved_in_tracing_section) {
		ctx->in_tracing_section = 0;
		goto end;
	}

	/* We can alter the packet */
	ctx->in_tracing_section = 1;

	/* Do not open a packet that is already open */
	if (ctx->packet_is_open) {
		ctx->in_tracing_section = saved_in_tracing_section;
		goto end;
	}

	ctx->at = 0;

	/* Write packet header structure */
	{
		/* Align for packet header structure */
		_ALIGN(ctx->at, 32);

		/* Align for `magic` field */
		_ALIGN(ctx->at, 32);

		/* Write magic number field */
		{
			const uint32_t tmp_val = (uint32_t) 0xc1fc1fc1UL;

			memcpy(&ctx->buf[_BITS_TO_BYTES(ctx->at)], &tmp_val, sizeof(tmp_val));
			ctx->at += 32;
		}

		/* Align for `stream_id` field */
		_ALIGN(ctx->at, 8);

		/* Write data stream type ID field */
		{
			const uint8_t tmp_val = (uint8_t) 0;

			memcpy(&ctx->buf[_BITS_TO_BYTES(ctx->at)], &tmp_val, sizeof(tmp_val));
			ctx->at += 8;
		}
	}

	/* Write packet context structure */
	{
		/* Align for packet context structure */
		_ALIGN(ctx->at, 32);

		/* Align for `packet_size` field */
		_ALIGN(ctx->at, 16);

		/* Write packet total size field */
		{
			const uint16_t tmp_val = (uint16_t) ctx->packet_size;

			memcpy(&ctx->buf[_BITS_TO_BYTES(ctx->at)], &tmp_val, sizeof(tmp_val));
			ctx->at += 16;
		}

		/* Align for `content_size` field */
		_ALIGN(ctx->at, 16);

		/* Do not write `content_size` field; save its offset */
		sctx->off_pc_content_size = ctx->at;
		ctx->at += 16;

		/* Align for `timestamp_begin` field */
		_ALIGN(ctx->at, 32);

		/* Write beginning timestamp field */
		{
			const uint32_t tmp_val = (uint32_t) ts;

			memcpy(&ctx->buf[_BITS_TO_BYTES(ctx->at)], &tmp_val, sizeof(tmp_val));
			ctx->at += 32;
		}

		/* Align for `timestamp_end` field */
		_ALIGN(ctx->at, 32);

		/* Do not write `timestamp_end` field; save its offset */
		sctx->off_pc_timestamp_end = ctx->at;
		ctx->at += 32;

		/* Align for `events_discarded` field */
		_ALIGN(ctx->at, 16);

		/* Do not write `events_discarded` field; save its offset */
		sctx->off_pc_events_discarded = ctx->at;
		ctx->at += 16;

		/* Align for `packet_seq_num` field */
		_ALIGN(ctx->at, 16);

		/* Write `packet_seq_num` field */
		{
			const uint16_t tmp_val = (uint16_t) ctx->sequence_number;

			memcpy(&ctx->buf[_BITS_TO_BYTES(ctx->at)], &tmp_val, sizeof(tmp_val));
			ctx->at += 16;
		}
	}

	/* Save content beginning's offset */
	ctx->off_content = ctx->at;

	/* Mark current packet as open */
	ctx->packet_is_open = 1;

	/* Not tracing anymore */
	ctx->in_tracing_section = saved_in_tracing_section;

end:
	return;
}

/* Close packet for data stream type `default` */
void barectf_default_close_packet(struct barectf_default_ctx * const sctx)
{
	struct barectf_ctx * const ctx = &sctx->parent;
	const uint32_t ts = ctx->use_cur_last_event_ts ?
		sctx->cur_last_event_ts :
		ctx->cbs.default_clock_get_value(ctx->data);
	const int saved_in_tracing_section = ctx->in_tracing_section;

	/*
	 * This function is either called by a tracing function, or
	 * directly by the platform.
	 *
	 * If it's called by a tracing function, then
	 * `ctx->in_tracing_section` is 1, so it's safe to close
	 * the packet here (alter the packet), even if tracing was
	 * disabled in the meantime, because we're already in a tracing
	 * section (which finishes at the end of the tracing function
	 * call).
	 *
	 * If it's called directly by the platform, then if tracing is
	 * disabled, we don't want to alter the packet, and return
	 * immediately.
	 */
	if (!ctx->is_tracing_enabled && !saved_in_tracing_section) {
		ctx->in_tracing_section = 0;
		goto end;
	}

	/* We can alter the packet */
	ctx->in_tracing_section = 1;

	/* Do not close a packet that is not open */
	if (!ctx->packet_is_open) {
		ctx->in_tracing_section = saved_in_tracing_section;
		goto end;
	}

	/* Save content size */
	ctx->content_size = ctx->at;

	/* Go back to `timestamp_end` field offset */
	ctx->at = sctx->off_pc_timestamp_end;

	/* Write `timestamp_end` field */
	{
		const uint32_t tmp_val = (uint32_t) ts;

		memcpy(&ctx->buf[_BITS_TO_BYTES(ctx->at)], &tmp_val, sizeof(tmp_val));
		ctx->at += 32;
	}

	/* Go back to `content_size` field offset */
	ctx->at = sctx->off_pc_content_size;

	/* Write `content_size` field */
	{
		const uint16_t tmp_val = (uint16_t) ctx->content_size;

		memcpy(&ctx->buf[_BITS_TO_BYTES(ctx->at)], &tmp_val, sizeof(tmp_val));
		ctx->at += 16;
	}

	/* Go back to `events_discarded` field offset */
	ctx->at = sctx->off_pc_events_discarded;

	/* Write `events_discarded` field */
	{
		const uint16_t tmp_val = (uint16_t) ctx->events_discarded;

		memcpy(&ctx->buf[_BITS_TO_BYTES(ctx->at)], &tmp_val, sizeof(tmp_val));
		ctx->at += 16;
	}

	/* Go back to end of packet */
	ctx->at = ctx->packet_size;

	/* Mark packet as closed */
	ctx->packet_is_open = 0;
	/* Increment sequence number for next packet */
	ctx->sequence_number++;

	/* Not tracing anymore */
	ctx->in_tracing_section = saved_in_tracing_section;

end:
	return;
}

static void _serialize_er_header_default(void * const vctx,
	const uint32_t ert_id)
{
	struct barectf_ctx * const ctx = _FROM_VOID_PTR(struct barectf_ctx, vctx);
	struct barectf_default_ctx * const sctx = _FROM_VOID_PTR(struct barectf_default_ctx, vctx);
	const uint32_t ts = sctx->cur_last_event_ts;

	/* Write header structure */
	{
		/* Align for header structure */
		_ALIGN(ctx->at, 32);

		/* Align for `id` field */
		_ALIGN(ctx->at, 8);

		/* Write event record type ID field */
		{
			const uint8_t tmp_val = (uint8_t) ert_id;

			memcpy(&ctx->buf[_BITS_TO_BYTES(ctx->at)], &tmp_val, sizeof(tmp_val));
			ctx->at += 8;
		}

		/* Align for `timestamp` field */
		_ALIGN(ctx->at, 32);

		/* Write timestamp field */
		{
			const uint32_t tmp_val = (uint32_t) ts;

			memcpy(&ctx->buf[_BITS_TO_BYTES(ctx->at)], &tmp_val, sizeof(tmp_val));
			ctx->at += 32;
		}
	}
}

static void _serialize_er_default_event_a(void * const vctx)
{
	struct barectf_ctx * const ctx = _FROM_VOID_PTR(struct barectf_ctx, vctx);

	/* Serialize header */
	_serialize_er_header_default(ctx, 0);
}

static void _serialize_er_default_event_b(void * const vctx,
	const uint32_t p_ms)
{
	struct barectf_ctx * const ctx = _FROM_VOID_PTR(struct barectf_ctx, vctx);

	/* Serialize header */
	_serialize_er_header_default(ctx, 1);

	/* Write payload structure */
	{
		/* Align for payload structure */
		_ALIGN(ctx->at, 32);

		/* Align for `ms` field */
		_ALIGN(ctx->at, 32);

		/* Write `ms` field */
		{
			const uint32_t tmp_val = (uint32_t) p_ms;

			memcpy(&ctx->buf[_BITS_TO_BYTES(ctx->at)], &tmp_val, sizeof(tmp_val));
			ctx->at += 32;
		}
	}
}

static uint32_t _er_size_default_event_a(void * const vctx)
{
	struct barectf_ctx * const ctx = _FROM_VOID_PTR(struct barectf_ctx, vctx);
	uint32_t at = ctx->at;

	/* Add header structure size */
	{
		/* Align for header structure */
		_ALIGN(at, 32);

		/* Align for `id` field */
		_ALIGN(at, 8);

		/* Add `id` bit array field's size */
		at += 8;

		/* Align for `timestamp` field */
		_ALIGN(at, 32);

		/* Add `timestamp` bit array field's size */
		at += 32;
	}

	return at - ctx->at;
}

static uint32_t _er_size_default_event_b(void * const vctx)
{
	struct barectf_ctx * const ctx = _FROM_VOID_PTR(struct barectf_ctx, vctx);
	uint32_t at = ctx->at;

	/* Add header structure size */
	{
		/* Align for header structure */
		_ALIGN(at, 32);

		/* Align for `id` field */
		_ALIGN(at, 8);

		/* Add `id` bit array field's size */
		at += 8;

		/* Align for `timestamp` field */
		_ALIGN(at, 32);

		/* Add `timestamp` bit array field's size */
		at += 32;
	}

	/* Add payload structure size */
	{
		/* Align for payload structure */
		_ALIGN(at, 32);

		/* Align for `ms` field */
		_ALIGN(at, 32);

		/* Add `ms` bit array field's size */
		at += 32;
	}

	return at - ctx->at;
}

/* Trace (data stream type `default`, event record type `event_a`) */
void barectf_default_trace_event_a(struct barectf_default_ctx * const sctx)
{
	struct barectf_ctx * const ctx = &sctx->parent;
	uint32_t er_size;

	/* Save timestamp */
	sctx->cur_last_event_ts = ctx->cbs.default_clock_get_value(ctx->data);

	if (!ctx->is_tracing_enabled) {
		goto end;
	}

	/* We can alter the packet */
	ctx->in_tracing_section = 1;

	/* Compute event record size */
	er_size = _er_size_default_event_a(_TO_VOID_PTR(ctx));

	/* Is there enough space to serialize? */
	if (!_reserve_er_space(_TO_VOID_PTR(ctx), er_size)) {
		/* no: forget this */
		ctx->in_tracing_section = 0;
		goto end;
	}

	/* Serialize event record */
	_serialize_er_default_event_a(_TO_VOID_PTR(ctx));

	/* Commit event record */
	_commit_er(_TO_VOID_PTR(ctx));

	/* Not tracing anymore */
	ctx->in_tracing_section = 0;

end:
	return;
}

/* Trace (data stream type `default`, event record type `event_b`) */
void barectf_default_trace_event_b(struct barectf_default_ctx * const sctx,
	const uint32_t p_ms)
{
	struct barectf_ctx * const ctx = &sctx->parent;
	uint32_t er_size;

	/* Save timestamp */
	sctx->cur_last_event_ts = ctx->cbs.default_clock_get_value(ctx->data);

	if (!ctx->is_tracing_enabled) {
		goto end;
	}

	/* We can alter the packet */
	ctx->in_tracing_section = 1;

	/* Compute event record size */
	er_size = _er_size_default_event_b(_TO_VOID_PTR(ctx));

	/* Is there enough space to serialize? */
	if (!_reserve_er_space(_TO_VOID_PTR(ctx), er_size)) {
		/* no: forget this */
		ctx->in_tracing_section = 0;
		goto end;
	}

	/* Serialize event record */
	_serialize_er_default_event_b(_TO_VOID_PTR(ctx), p_ms);

	/* Commit event record */
	_commit_er(_TO_VOID_PTR(ctx));

	/* Not tracing anymore */
	ctx->in_tracing_section = 0;

end:
	return;
}