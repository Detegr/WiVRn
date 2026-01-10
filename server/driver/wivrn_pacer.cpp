/*
 * WiVRn VR streaming
 * Copyright (C) 2024  Guillaume Meunier <guillaume.meunier@centraliens.net>
 * Copyright (C) 2024  Patrick Nicolas <patricknicolas@laposte.net>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "wivrn_pacer.h"
#include "driver/clock_offset.h"
#include "os/os_time.h"
#include "util/u_logging.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace wivrn
{

static const int64_t margin_ns = 3'000'000;
static const int64_t slop_ns = 500'000;

// Enable with WIVRN_PACER_DEBUG=1
static bool pacer_debug_enabled()
{
	static int enabled = -1;
	if (enabled < 0)
	{
		const char * env = std::getenv("WIVRN_PACER_DEBUG");
		enabled = (env && env[0] == '1') ? 1 : 0;
	}
	return enabled == 1;
}

#define PACER_DEBUG(...) \
	do { if (pacer_debug_enabled()) U_LOG_I(__VA_ARGS__); } while(0)

template <typename T>
static T lerp_mod(T a, T b, double t, T mod)
{
	if (2 * std::abs(a - b) < mod)
		return std::lerp(a, b, t);
	if (a < b)
		a += mod;
	else
		b += mod;
	return T(std::lerp(a, b, t)) % mod;
}

wivrn_pacer::wivrn_pacer(uint64_t frame_duration) :
        frame_duration_ns(frame_duration),
        frame_times(5000),
        frame_times_compute(frame_times),
        worker([this](std::stop_token t) {
	        std::vector<XrDuration> samples;
	        while (not t.stop_requested())
	        {
		        samples.clear();
		        {
			        std::unique_lock lock(compute_mutex);
			        compute_cv.wait(lock);
			        for (const auto & time: frame_times_compute)
			        {
				        if (time.decoded > time.present)
					        samples.push_back(time.decoded - time.present);
			        }
		        }
		        if (samples.empty())
			        continue;
		        auto it = samples.begin() + (samples.size() * 995) / 1000;
		        std::ranges::nth_element(samples, it);

		        std::unique_lock lock(mutex);
		        safe_present_to_decoded_ns = *it + 1'000'000;
	        }
        })
{}

wivrn_pacer::~wivrn_pacer()
{
	worker.request_stop();
	compute_cv.notify_all();
}

void wivrn_pacer::set_frame_duration(uint64_t frame_duration_ns)
{
	std::lock_guard lock(mutex);
	this->frame_duration_ns = frame_duration_ns;
}

void wivrn_pacer::predict(
        int64_t & frame_id,
        int64_t & out_wake_up_time_ns,
        int64_t & out_desired_present_time_ns,
        int64_t & out_present_slop_ns,
        int64_t & out_predicted_display_time_ns)
{
	std::lock_guard lock(mutex);
	frame_id = this->frame_id++;
	auto now = os_monotonic_get_ns();

	int64_t predicted_client_render = last_ns + frame_duration_ns;
	int64_t predicted_before_snap = predicted_client_render;
	// snap to phase
	predicted_client_render = (predicted_client_render / frame_duration_ns) * frame_duration_ns + client_render_phase_ns;
	int64_t predicted_after_snap = predicted_client_render;

	int64_t frames_to_skip = 0;
	int64_t late_by_ns = (now + mean_wake_up_to_present_ns + safe_present_to_decoded_ns) - predicted_client_render;
	if (late_by_ns > 0)
	{
		// Use ceiling division to ensure we skip at least 1 frame when late
		// This fixes the case where we're almost-but-not-quite one frame late
		// (e.g., 11.0ms late with 11.1ms frame period would truncate to 0 with floor division)
		frames_to_skip = (late_by_ns + frame_duration_ns - 1) / frame_duration_ns;
		predicted_client_render += frame_duration_ns * frames_to_skip;
	}

	out_predicted_display_time_ns = predicted_client_render + mean_render_to_display_ns;
	out_desired_present_time_ns = predicted_client_render - safe_present_to_decoded_ns;
	out_wake_up_time_ns = out_desired_present_time_ns - mean_wake_up_to_present_ns + margin_ns; // we should be awoken early by the application
	last_wake_up_ns = out_wake_up_time_ns;

	last_ns = predicted_client_render;

	in_flight_frames[frame_id % in_flight_frames.size()] = {
	        .frame_id = frame_id,
	        .present_ns = out_desired_present_time_ns,
	        .predicted_display_time = out_predicted_display_time_ns,
	};

	out_present_slop_ns = slop_ns;

	// Diagnostic logging
	PACER_DEBUG("PREDICT frame=%ld now=%.2fms last_ns=%.2fms phase=%.2fms",
	            frame_id,
	            now / 1e6,
	            (last_ns - frame_duration_ns) / 1e6,  // previous last_ns
	            client_render_phase_ns / 1e6);
	PACER_DEBUG("  before_snap=%.2fms after_snap=%.2fms late_by=%.2fms frames_skipped=%ld final=%.2fms",
	            predicted_before_snap / 1e6,
	            predicted_after_snap / 1e6,
	            late_by_ns / 1e6,
	            frames_to_skip,
	            predicted_client_render / 1e6);
	PACER_DEBUG("  margins: wake_to_present=%.2fms safe_decode=%.2fms render_to_display=%.2fms",
	            mean_wake_up_to_present_ns / 1e6,
	            safe_present_to_decoded_ns / 1e6,
	            mean_render_to_display_ns / 1e6);
	PACER_DEBUG("  output: desired_present=%.2fms predicted_display=%.2fms wake_up=%.2fms",
	            out_desired_present_time_ns / 1e6,
	            out_predicted_display_time_ns / 1e6,
	            out_wake_up_time_ns / 1e6);
}

void wivrn_pacer::on_feedback(const wivrn::from_headset::feedback & feedback, const clock_offset & offset)
{
	if (feedback.times_displayed > 1 or not feedback.blitted)
		return;

	std::lock_guard lock(mutex);
	auto & when = in_flight_frames[feedback.frame_index % in_flight_frames.size()];
	if (when.frame_id != feedback.frame_index)
	{
		PACER_DEBUG("FEEDBACK frame=%ld STALE (stored=%ld)",
		            feedback.frame_index, when.frame_id);
		return;
	}

	auto & times = frame_times[feedback.frame_index % frame_times.size()];
	if (times.frame_id != feedback.frame_index)
	{
		times.frame_id = feedback.frame_index;
		times.present = when.present_ns;
		times.decoded = 0;
	}
	int64_t decoded_server_time = offset.from_headset(feedback.received_from_decoder);
	times.decoded = std::max(times.decoded, decoded_server_time);

	// Calculate timing differences
	int64_t present_to_decoded_diff = times.decoded - times.present;

	if (feedback.stream_index == 0)
	{
		if (feedback.frame_index % 100 == 0)
		{
			std::unique_lock lock(compute_mutex);
			std::swap(frame_times, frame_times_compute);
			compute_cv.notify_all();
		}

		int64_t old_phase = client_render_phase_ns;
		int64_t blitted_server_time = offset.from_headset(feedback.blitted);
		int64_t new_phase_sample = blitted_server_time % frame_duration_ns;
		client_render_phase_ns = lerp_mod<int64_t>(client_render_phase_ns, new_phase_sample, 0.1, frame_duration_ns);

		// Log phase changes periodically or when significant
		int64_t phase_change = client_render_phase_ns - old_phase;
		if (phase_change < 0) phase_change = -phase_change;
		if (phase_change > frame_duration_ns / 2)
			phase_change = frame_duration_ns - phase_change;  // Handle wraparound

		if (feedback.frame_index % 90 == 0 || phase_change > 500'000)  // Log every ~1s or big changes
		{
			PACER_DEBUG("FEEDBACK frame=%ld stream=%d present_ns=%.2fms decoded=%.2fms diff=%.2fms",
			            feedback.frame_index,
			            feedback.stream_index,
			            when.present_ns / 1e6,
			            decoded_server_time / 1e6,
			            present_to_decoded_diff / 1e6);
			PACER_DEBUG("  phase: old=%.2fms sample=%.2fms new=%.2fms change=%.3fms",
			            old_phase / 1e6,
			            new_phase_sample / 1e6,
			            client_render_phase_ns / 1e6,
			            phase_change / 1e6);
			PACER_DEBUG("  blitted=%.2fms displayed=%.2fms (headset times converted)",
			            blitted_server_time / 1e6,
			            feedback.displayed ? offset.from_headset(feedback.displayed) / 1e6 : 0.0);
		}
	}

	if (feedback.displayed and feedback.displayed > feedback.blitted and feedback.displayed < feedback.blitted + 100'000'000)
	{
		int64_t old_render_to_display = mean_render_to_display_ns;
		mean_render_to_display_ns = std::lerp(mean_render_to_display_ns, feedback.displayed - feedback.blitted, 0.1);

		if (feedback.frame_index % 90 == 0)
		{
			PACER_DEBUG("  render_to_display: sample=%.2fms mean=%.2fms (was %.2fms)",
			            (feedback.displayed - feedback.blitted) / 1e6,
			            mean_render_to_display_ns / 1e6,
			            old_render_to_display / 1e6);
		}
	}
}
void wivrn_pacer::mark_timing_point(
        comp_target_timing_point point,
        int64_t frame_id,
        int64_t when_ns)
{
	switch (point)
	{
		//! Woke up after sleeping in wait frame.
		case COMP_TARGET_TIMING_POINT_WAKE_UP:
			PACER_DEBUG("TIMING frame=%ld WAKE_UP when=%.2fms", frame_id, when_ns / 1e6);
			return;

		//! Began CPU side work for GPU.
		case COMP_TARGET_TIMING_POINT_BEGIN:
			PACER_DEBUG("TIMING frame=%ld BEGIN when=%.2fms", frame_id, when_ns / 1e6);
			return;

		//! Just before submitting work to the GPU.
		case COMP_TARGET_TIMING_POINT_SUBMIT_BEGIN:
			PACER_DEBUG("TIMING frame=%ld SUBMIT_BEGIN when=%.2fms", frame_id, when_ns / 1e6);
			return;

		//! Just after submitting work to the GPU.
		case COMP_TARGET_TIMING_POINT_SUBMIT_END:
			if (when_ns > last_wake_up_ns and when_ns < last_wake_up_ns + 100'000'000)
			{
				std::lock_guard lock(mutex);
				int64_t old_wake_to_present = mean_wake_up_to_present_ns;
				int64_t sample = when_ns - last_wake_up_ns;
				mean_wake_up_to_present_ns = std::lerp(mean_wake_up_to_present_ns, sample, 0.1);
				PACER_DEBUG("TIMING frame=%ld SUBMIT_END when=%.2fms wake_to_present: sample=%.2fms mean=%.2fms (was %.2fms)",
				            frame_id, when_ns / 1e6, sample / 1e6, mean_wake_up_to_present_ns / 1e6, old_wake_to_present / 1e6);
			}
			else
			{
				PACER_DEBUG("TIMING frame=%ld SUBMIT_END when=%.2fms (REJECTED: outside wake window)",
				            frame_id, when_ns / 1e6);
			}
	}
}

wivrn_pacer::frame_info wivrn_pacer::present_to_info(int64_t present)
{
	std::lock_guard lock(mutex);
	for (size_t i = 0; i < in_flight_frames.size(); i++)
	{
		const auto & info = in_flight_frames[i];
		if (info.present_ns == present)
		{
			PACER_DEBUG("LOOKUP present=%.2fms -> frame=%ld display=%.2fms (slot %zu)",
			            present / 1e6, info.frame_id, info.predicted_display_time / 1e6, i);
			return info;
		}
	}

	// Lookup failed - log all slots for debugging
	U_LOG_W("LOOKUP FAILED present=%.2fms - dumping all slots:", present / 1e6);
	for (size_t i = 0; i < in_flight_frames.size(); i++)
	{
		const auto & info = in_flight_frames[i];
		U_LOG_W("  slot %zu: frame=%ld present=%.2fms display=%.2fms diff=%.2fms",
		        i, info.frame_id, info.present_ns / 1e6, info.predicted_display_time / 1e6,
		        (present - info.present_ns) / 1e6);
	}
	assert(false);
	return {};
}

void wivrn_pacer::reset()
{
	std::lock_guard lock(mutex);
	std::ranges::fill(frame_times, frame_time{});
}
} // namespace wivrn
