# SPDX-License-Identifier: Apache-2.0
set(BOARD xiao_ble/nrf52840/sense)
cmake_minimum_required(VERSION 3.20.0)
set(KCONFIG_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/Kconfig)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(xiao_minimal)
enable_language(C ASM)

target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../devkit/src
)

target_sources(app PRIVATE
  src/main.c
  src/transport.c
  src/stubs.c

  ../devkit/src/mic.c
  ../devkit/src/codec.c
  ../devkit/src/led.c
  ../devkit/src/button.c
  ../devkit/src/usb.c
  ../devkit/src/wdog_facade.c
  ../devkit/src/lib/battery/battery.c
)

target_sources_ifdef(CONFIG_OMI_CODEC_OPUS app PRIVATE
  ../devkit/src/lib/opus-1.2.1/A2NLSF.c
  ../devkit/src/lib/opus-1.2.1/CNG.c
  ../devkit/src/lib/opus-1.2.1/HP_variable_cutoff.c
  ../devkit/src/lib/opus-1.2.1/LPC_analysis_filter.c
  ../devkit/src/lib/opus-1.2.1/LPC_fit.c
  ../devkit/src/lib/opus-1.2.1/LPC_inv_pred_gain.c
  ../devkit/src/lib/opus-1.2.1/LP_variable_cutoff.c
  ../devkit/src/lib/opus-1.2.1/LTP_analysis_filter_FIX.c
  ../devkit/src/lib/opus-1.2.1/LTP_scale_ctrl_FIX.c
  ../devkit/src/lib/opus-1.2.1/NLSF2A.c
  ../devkit/src/lib/opus-1.2.1/NLSF_VQ.c
  ../devkit/src/lib/opus-1.2.1/NLSF_VQ_weights_laroia.c
  ../devkit/src/lib/opus-1.2.1/NLSF_decode.c
  ../devkit/src/lib/opus-1.2.1/NLSF_del_dec_quant.c
  ../devkit/src/lib/opus-1.2.1/NLSF_encode.c
  ../devkit/src/lib/opus-1.2.1/NLSF_stabilize.c
  ../devkit/src/lib/opus-1.2.1/NLSF_unpack.c
  ../devkit/src/lib/opus-1.2.1/NSQ.c
  ../devkit/src/lib/opus-1.2.1/NSQ_del_dec.c
  ../devkit/src/lib/opus-1.2.1/PLC.c
  ../devkit/src/lib/opus-1.2.1/VAD.c
  ../devkit/src/lib/opus-1.2.1/VQ_WMat_EC.c
  ../devkit/src/lib/opus-1.2.1/ana_filt_bank_1.c
  ../devkit/src/lib/opus-1.2.1/analysis.c
  ../devkit/src/lib/opus-1.2.1/apply_sine_window_FIX.c
  ../devkit/src/lib/opus-1.2.1/autocorr_FIX.c
  ../devkit/src/lib/opus-1.2.1/bands.c
  ../devkit/src/lib/opus-1.2.1/biquad_alt.c
  ../devkit/src/lib/opus-1.2.1/burg_modified_FIX.c
  ../devkit/src/lib/opus-1.2.1/bwexpander.c
  ../devkit/src/lib/opus-1.2.1/bwexpander_32.c
  ../devkit/src/lib/opus-1.2.1/celt.c
  ../devkit/src/lib/opus-1.2.1/celt_decoder.c
  ../devkit/src/lib/opus-1.2.1/celt_encoder.c
  ../devkit/src/lib/opus-1.2.1/celt_lpc.c
  ../devkit/src/lib/opus-1.2.1/arm/celt_pitch_xcorr_arm_gcc.s
  ../devkit/src/lib/opus-1.2.1/check_control_input.c
  ../devkit/src/lib/opus-1.2.1/code_signs.c
  ../devkit/src/lib/opus-1.2.1/control_SNR.c
  ../devkit/src/lib/opus-1.2.1/control_audio_bandwidth.c
  ../devkit/src/lib/opus-1.2.1/control_codec.c
  ../devkit/src/lib/opus-1.2.1/corrMatrix_FIX.c
  ../devkit/src/lib/opus-1.2.1/cwrs.c
  ../devkit/src/lib/opus-1.2.1/debug.c
  ../devkit/src/lib/opus-1.2.1/dec_API.c
  ../devkit/src/lib/opus-1.2.1/decode_core.c
  ../devkit/src/lib/opus-1.2.1/decode_frame.c
  ../devkit/src/lib/opus-1.2.1/decode_indices.c
  ../devkit/src/lib/opus-1.2.1/decode_parameters.c
  ../devkit/src/lib/opus-1.2.1/decode_pitch.c
  ../devkit/src/lib/opus-1.2.1/decode_pulses.c
  ../devkit/src/lib/opus-1.2.1/decoder_set_fs.c
  ../devkit/src/lib/opus-1.2.1/enc_API.c
  ../devkit/src/lib/opus-1.2.1/encode_frame_FIX.c
  ../devkit/src/lib/opus-1.2.1/encode_indices.c
  ../devkit/src/lib/opus-1.2.1/encode_pulses.c
  ../devkit/src/lib/opus-1.2.1/entcode.c
  ../devkit/src/lib/opus-1.2.1/entdec.c
  ../devkit/src/lib/opus-1.2.1/entenc.c
  ../devkit/src/lib/opus-1.2.1/find_LPC_FIX.c
  ../devkit/src/lib/opus-1.2.1/find_LTP_FIX.c
  ../devkit/src/lib/opus-1.2.1/find_pitch_lags_FIX.c
  ../devkit/src/lib/opus-1.2.1/find_pred_coefs_FIX.c
  ../devkit/src/lib/opus-1.2.1/gain_quant.c
  ../devkit/src/lib/opus-1.2.1/init_decoder.c
  ../devkit/src/lib/opus-1.2.1/init_encoder.c
  ../devkit/src/lib/opus-1.2.1/inner_prod_aligned.c
  ../devkit/src/lib/opus-1.2.1/interpolate.c
  ../devkit/src/lib/opus-1.2.1/k2a_FIX.c
  ../devkit/src/lib/opus-1.2.1/k2a_Q16_FIX.c
  ../devkit/src/lib/opus-1.2.1/kiss_fft.c
  ../devkit/src/lib/opus-1.2.1/laplace.c
  ../devkit/src/lib/opus-1.2.1/lin2log.c
  ../devkit/src/lib/opus-1.2.1/log2lin.c
  ../devkit/src/lib/opus-1.2.1/mathops.c
  ../devkit/src/lib/opus-1.2.1/mdct.c
  ../devkit/src/lib/opus-1.2.1/mlp.c
  ../devkit/src/lib/opus-1.2.1/mlp_data.c
  ../devkit/src/lib/opus-1.2.1/modes.c
  ../devkit/src/lib/opus-1.2.1/noise_shape_analysis_FIX.c
  ../devkit/src/lib/opus-1.2.1/opus.c
  ../devkit/src/lib/opus-1.2.1/opus_decoder.c
  ../devkit/src/lib/opus-1.2.1/opus_encoder.c
  ../devkit/src/lib/opus-1.2.1/opus_multistream.c
  ../devkit/src/lib/opus-1.2.1/opus_multistream_decoder.c
  ../devkit/src/lib/opus-1.2.1/opus_multistream_encoder.c
  ../devkit/src/lib/opus-1.2.1/pitch.c
  ../devkit/src/lib/opus-1.2.1/pitch_analysis_core_FIX.c
  ../devkit/src/lib/opus-1.2.1/pitch_est_tables.c
  ../devkit/src/lib/opus-1.2.1/process_NLSFs.c
  ../devkit/src/lib/opus-1.2.1/process_gains_FIX.c
  ../devkit/src/lib/opus-1.2.1/quant_LTP_gains.c
  ../devkit/src/lib/opus-1.2.1/quant_bands.c
  ../devkit/src/lib/opus-1.2.1/rate.c
  ../devkit/src/lib/opus-1.2.1/regularize_correlations_FIX.c
  ../devkit/src/lib/opus-1.2.1/repacketizer.c
  ../devkit/src/lib/opus-1.2.1/resampler.c
  ../devkit/src/lib/opus-1.2.1/resampler_down2.c
  ../devkit/src/lib/opus-1.2.1/resampler_down2_3.c
  ../devkit/src/lib/opus-1.2.1/resampler_private_AR2.c
  ../devkit/src/lib/opus-1.2.1/resampler_private_IIR_FIR.c
  ../devkit/src/lib/opus-1.2.1/resampler_private_down_FIR.c
  ../devkit/src/lib/opus-1.2.1/resampler_private_up2_HQ.c
  ../devkit/src/lib/opus-1.2.1/resampler_rom.c
  ../devkit/src/lib/opus-1.2.1/residual_energy16_FIX.c
  ../devkit/src/lib/opus-1.2.1/residual_energy_FIX.c
  ../devkit/src/lib/opus-1.2.1/schur64_FIX.c
  ../devkit/src/lib/opus-1.2.1/schur_FIX.c
  ../devkit/src/lib/opus-1.2.1/shell_coder.c
  ../devkit/src/lib/opus-1.2.1/sigm_Q15.c
  ../devkit/src/lib/opus-1.2.1/sort.c
  ../devkit/src/lib/opus-1.2.1/stereo_LR_to_MS.c
  ../devkit/src/lib/opus-1.2.1/stereo_MS_to_LR.c
  ../devkit/src/lib/opus-1.2.1/stereo_decode_pred.c
  ../devkit/src/lib/opus-1.2.1/stereo_encode_pred.c
  ../devkit/src/lib/opus-1.2.1/stereo_find_predictor.c
  ../devkit/src/lib/opus-1.2.1/stereo_quant_pred.c
  ../devkit/src/lib/opus-1.2.1/sum_sqr_shift.c
  ../devkit/src/lib/opus-1.2.1/table_LSF_cos.c
  ../devkit/src/lib/opus-1.2.1/tables_LTP.c
  ../devkit/src/lib/opus-1.2.1/tables_NLSF_CB_NB_MB.c
  ../devkit/src/lib/opus-1.2.1/tables_NLSF_CB_WB.c
  ../devkit/src/lib/opus-1.2.1/tables_gain.c
  ../devkit/src/lib/opus-1.2.1/tables_other.c
  ../devkit/src/lib/opus-1.2.1/tables_pitch_lag.c
  ../devkit/src/lib/opus-1.2.1/tables_pulses_per_block.c
  ../devkit/src/lib/opus-1.2.1/vector_ops_FIX.c
  ../devkit/src/lib/opus-1.2.1/vq.c
  ../devkit/src/lib/opus-1.2.1/warped_autocorrelation_FIX.c
)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DARM_MATH_CM4")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DOPUS_ARM_ASM")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DOPUS_ARM_INLINE_ASM")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DOPUS_ARM_INLINE_EDSP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DOPUS_ARM_INLINE_MEDIA")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DOPUS_ARM_MAY_HAVE_EDSP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DOPUS_ARM_PRESUME_EDSP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DOPUS_BUILD")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DUSE_ALLOCA")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DFIXED_POINT -DDISABLE_FLOAT_API")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DHAVE_CONFIG_H")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DHAVE_ALLOCA_H")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsingle-precision-constant")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DHAVE_LRINT -DHAVE_LRINTF")
