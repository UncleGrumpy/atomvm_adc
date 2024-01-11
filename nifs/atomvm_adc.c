// Copyright (c) 2020-2023 dushin.net
// Copyright (c) 2022-2024 Winford <winford@object.stream>
// Copyright (c) 2024 Ricardo Lanziano <arpunk@fatelectron.net>
// All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

// References
// https://docs.espressif.com/projects/esp-idf/en/v5.0.7/esp32/api-reference/peripherals/adc_oneshot.html
// https://docs.espressif.com/projects/esp-idf/en/v5.0.7/esp32/api-reference/peripherals/adc_calibration.html
// https://docs.espressif.com/projects/esp-idf/en/v5.3/esp32/api-reference/peripherals/adc_oneshot.html
// https://docs.espressif.com/projects/esp-idf/en/v5.3/esp32/api-reference/peripherals/adc_calibration.html

#include "atomvm_adc.h"

#include <esp32_sys.h>

#include <context.h>
#include <defaultatoms.h>
#include <erl_nif_priv.h>
#include <globalcontext.h>
#include <interop.h>
#include <nifs.h>
#include <term.h>

// #define ENABLE_TRACE
#include <trace.h>

#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_log.h>
#include <hal/adc_types.h>
#include <soc/soc_caps.h>
#include <sdkconfig.h>

#include <math.h>
#include <stdlib.h>

#define TAG "atomvm_adc"
#define DEFAULT_SAMPLES 64
#define DEFAULT_VREF 1100
#define ADC_INVALID_PARAM -1

typedef enum avm_calibration_method
{
    UNCALIBRATED,
    ESTIMATED,
    CURVE,
    LINE,
} cali_method_t;

struct ChannelResource
{
    adc_atten_t attenuation;
    adc_bitwidth_t width;
    adc_unit_t adc_unit;
    adc_channel_t channel;
    adc_cali_handle_t cali_handle;
    cali_method_t calibration;
};

struct UnitResource
{
    adc_oneshot_unit_handle_t unit_handle;
#ifdef CONFIG_AVM_ADC2_ENABLE
    adc_oneshot_unit_handle_t unit2_handle;
#endif
};

struct SampleValue
{
    term raw;
    term mv;
};

static const AtomStringIntPair bit_width_table[] = {
    { ATOM_STR("\x7", "bit_max"), ADC_BITWIDTH_DEFAULT },
#if SOC_ADC_MAX_BITWIDTH == 13
    { ATOM_STR("\x6", "bit_13"), ADC_BITWIDTH_13 },
#elif SOC_ADC_MAX_BITWIDTH == 12
    { ATOM_STR("\x6", "bit_12"), ADC_BITWIDTH_12 },
#elif CONFIG_IDF_TARGET_ESP32
    { ATOM_STR("\x6", "bit_11"), ADC_BITWIDTH_11 },
    { ATOM_STR("\x6", "bit_10"), ADC_BITWIDTH_10 },
    { ATOM_STR("\x5", "bit_9"), ADC_BITWIDTH_9 },
#endif
    SELECT_INT_DEFAULT(ADC_INVALID_PARAM)
};

static const AtomStringIntPair attenuation_table[] = {
    { ATOM_STR("\x4", "db_0"), ADC_ATTEN_DB_0 },
    { ATOM_STR("\x6", "db_2_5"), ADC_ATTEN_DB_2_5 },
    { ATOM_STR("\x4", "db_6"), ADC_ATTEN_DB_6 },
    { ATOM_STR("\x5", "db_11"), ADC_ATTEN_DB_11 },
#ifdef ADC_ATTEN_DB_12
    { ATOM_STR("\x5", "db_12"), ADC_ATTEN_DB_12 },
#endif
    SELECT_INT_DEFAULT(ADC_INVALID_PARAM)
};

static ErlNifResourceType *adc_unit_resource;
static ErlNifResourceType *adc_channel_resource;

static bool is_adc_resource(GlobalContext *global, term t)
{
    bool ret = term_is_tuple(t)
        && term_get_tuple_arity(t) == 3
        && globalcontext_is_term_equal_to_atom_string(global, term_get_tuple_element(t, 0), ATOM_STR("\x4", "$adc"))
        && term_is_binary(term_get_tuple_element(t, 1))
        && term_is_reference(term_get_tuple_element(t, 2));

    return ret;
}

static bool to_channel_resource(term chan_resource, struct ChannelResource **rsrc_obj, Context *ctx)
{
    if (!is_adc_resource(ctx->global, chan_resource)) {
        return false;
    }
    void *rsrc_obj_ptr;
    if (UNLIKELY(!enif_get_resource(erl_nif_env_from_context(ctx), term_get_tuple_element(chan_resource, 1), adc_channel_resource, &rsrc_obj_ptr))) {
        return false;
    }
    *rsrc_obj = (struct ChannelResource *) rsrc_obj_ptr;

    return true;
}

static bool to_unit_resource(term unit_resource, struct UnitResource **rsrc_obj, Context *ctx)
{
    if (!is_adc_resource(ctx->global, unit_resource)) {
        return false;
    }
    void *rsrc_obj_ptr;
    if (UNLIKELY(!enif_get_resource(erl_nif_env_from_context(ctx), term_get_tuple_element(unit_resource, 1), adc_unit_resource, &rsrc_obj_ptr))) {
        return false;
    }
    *rsrc_obj = (struct UnitResource *) rsrc_obj_ptr;

    return true;
}

static int approximate_millivolts(int adc_reading, adc_atten_t attenuation, adc_bitwidth_t width)
{
    int digi_max = (int) pow(2, width);  // casting double to int here is safe because values are always between 512-8192
    int millivolt_max = 0;

    switch (attenuation) {
        case ADC_ATTEN_DB_0:
            millivolt_max = 950;
            break;
        case ADC_ATTEN_DB_2_5:
            millivolt_max = 1250;
            break;
        case ADC_ATTEN_DB_6:
            millivolt_max = 1750;
            break;
        case ADC_ATTEN_DB_11:
            millivolt_max = 2450;
            break;
#ifdef ADC_ATTEN_DB_12
        case ADC_ATTEN_DB_12:
            millivolt_max = 2450;
            break;
#endif
    }

    // Estimate V = RAW * ATTEN_MAX_MV / 2^BITWIDTH
    // See: https://docs.espressif.com/projects/esp-idf/en/v5.2.2/esp32/api-reference/peripherals/adc_oneshot.html#read-conversion-result

    return (adc_reading * millivolt_max) / digi_max;
}

static term create_pair(Context *ctx, term term1, term term2)
{
    term ret = term_alloc_tuple(2, &ctx->heap);
    term_put_tuple_element(ret, 0, term1);
    term_put_tuple_element(ret, 1, term2);

    return ret;
}

static cali_method_t do_adc_calibration(adc_unit_t unit, adc_channel_t chan, adc_atten_t atten, adc_bitwidth_t width, adc_cali_handle_t *cali_handle)
{
    cali_method_t calibration = UNCALIBRATED;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = unit,
        .chan = chan,
        .atten = atten,
        .bitwidth = width,
    };
    esp_err_t err = adc_cali_create_scheme_curve_fitting(&cali_config, cali_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to calibrate using the supported curve fitting scheme: %s", esp_err_to_name(err));
        ESP_LOGW(TAG, "any reading requesting 'voltage' will receive an estimated value");
    } else {
        calibration = CURVE;
        ESP_LOGD(TAG, "Characterized using curve fitting scheme");
    }
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = unit,
        .atten = atten,
        .bitwidth = width,
    };
    esp_err_t err = adc_cali_create_scheme_line_fitting(&cali_config, cali_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to calibrate using the supported line fitting scheme: %s", esp_err_to_name(err));
        ESP_LOGW(TAG, "any reading requesting 'voltage' will receive an estimated value");
    } else {
        calibration = LINE;
        ESP_LOGD(TAG, "Characterized using line fitting scheme");
    }
#else
    calibration = ESTIMATED;
    ESP_LOGD(TAG, "No supported calibration method, readings requesting 'voltage' will receive an estimated value");
#endif

    return calibration;
}

static term nif_adc_init(Context *ctx, int argc, term argv[])
{
    UNUSED(argc);
    UNUSED(argv);

    struct UnitResource *unit_rsrc = enif_alloc_resource(adc_unit_resource, sizeof(struct UnitResource));
    if (IS_NULL_PTR(unit_rsrc)) {
        ESP_LOGE(TAG, "failed to allocate resource: %s:%i.", __FILE__, __LINE__);
        RAISE_ERROR(OUT_OF_MEMORY_ATOM);
    }
    unit_rsrc->unit_handle = NULL;

    adc_unit_t adc_unit = ADC_UNIT_1;

    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = adc_unit,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    esp_err_t err = adc_oneshot_new_unit(&init_config, &unit_rsrc->unit_handle);
    if (UNLIKELY(err != ESP_OK)) {
        RAISE_ERROR(globalcontext_make_atom(ctx->global, (esp_err_to_name(err))));
    }

#ifdef CONFIG_AVM_ADC2_ENABLE
    unit_rsrc->unit2_handle = NULL;

    adc_unit = ADC_UNIT_2;

    adc_oneshot_unit_init_cfg_t init_config2 = {
        .unit_id = adc_unit,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    err = adc_oneshot_new_unit(&init_config2, &unit_rsrc->unit2_handle);
    if (UNLIKELY(err != ESP_OK)) {
        RAISE_ERROR(globalcontext_make_atom(ctx->global, (esp_err_to_name(err))));
    }
#endif

    if (UNLIKELY(memory_ensure_free(ctx, TERM_BOXED_RESOURCE_SIZE) != MEMORY_GC_OK)) {
        enif_release_resource(unit_rsrc);
        ESP_LOGE(TAG, "failed to allocate memory for resource: %s:%i.", __FILE__, __LINE__);
        RAISE_ERROR(OUT_OF_MEMORY_ATOM);
    }
    ERL_NIF_TERM unit_obj = enif_make_resource(erl_nif_env_from_context(ctx), unit_rsrc);
    enif_release_resource(unit_rsrc);

    // {ok, {'$adc', Unit :: resource(), ref()}}
    size_t requested_size = TUPLE_SIZE(2) + TUPLE_SIZE(3) + REF_SIZE;
    ESP_LOGD(TAG, "Requesting memory size %u for return message", requested_size);
    if (UNLIKELY(memory_ensure_free_with_roots(ctx, requested_size, 1, &unit_obj, MEMORY_CAN_SHRINK) != MEMORY_GC_OK)) {
        ESP_LOGE(TAG, "failed to allocate tuple memory size %u: %s:%i.", requested_size, __FILE__, __LINE__);
        RAISE_ERROR(OUT_OF_MEMORY_ATOM);
    }
    term unit_resource = term_alloc_tuple(3, &ctx->heap);
    term_put_tuple_element(unit_resource, 0, globalcontext_make_atom(ctx->global, ATOM_STR("\x4", "$adc")));
    term_put_tuple_element(unit_resource, 1, unit_obj);
    uint64_t ref_ticks = globalcontext_get_ref_ticks(ctx->global);
    term ref = term_from_ref_ticks(ref_ticks, &ctx->heap);
    term_put_tuple_element(unit_resource, 2, ref);

    term ret = term_alloc_tuple(2, &ctx->heap);
    term_put_tuple_element(ret, 0, OK_ATOM);
    term_put_tuple_element(ret, 1, unit_resource);

    return ret;
}

static term nif_adc_deinit(Context *ctx, int argc, term argv[])
{
    term unit_term = argv[0];
    if (UNLIKELY(!is_adc_resource(ctx->global, unit_term))) {
        ESP_LOGE(TAG, "handle supplied is not a valid adc resource");
        RAISE_ERROR(BADARG_ATOM);
    }
    struct UnitResource *unit_rsrc = NULL;
    if (UNLIKELY(!to_unit_resource(unit_term, &unit_rsrc, ctx))) {
        ESP_LOGE(TAG, "resource supplied is not a valid adc unit resource");
        RAISE_ERROR(BADARG_ATOM);
    }

    esp_err_t err = adc_oneshot_del_unit(unit_rsrc->unit_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to release ADC Unit");
        RAISE_ERROR(globalcontext_make_atom(ctx->global, (esp_err_to_name(err))));
    }
 #ifdef CONFIG_AVM_ADC2_ENABLE
    err = adc_oneshot_del_unit(unit_rsrc->unit2_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to release ADC Unit 2");
    }
#endif
    resource_type_destroy(adc_unit_resource);
    ESP_LOGD(TAG, "ADC unit released");

    return OK_ATOM;
}

static term nif_adc_acquire(Context *ctx, int argc, term argv[])
{
    UNUSED(argc);
    term pin = argv[0];
    VALIDATE_VALUE(pin, term_is_integer);
    int pin_num = term_to_int(pin);

    adc_unit_t adc_unit;
    adc_channel_t adc_channel;

    esp_err_t err = adc_oneshot_io_to_channel(pin_num, &adc_unit, &adc_channel);
    if (UNLIKELY(err != ESP_OK)) {
        ESP_LOGE(TAG, "pin %i does not support ADC peripheral", pin_num);
        RAISE_ERROR(BADARG_ATOM);
    }

    term width = argv[1];
    VALIDATE_VALUE(width, term_is_atom);
    int bits = interop_atom_term_select_int(bit_width_table, width, ctx->global);
    if (UNLIKELY(bits == ADC_INVALID_PARAM )) {
        ESP_LOGE(TAG, "invalid bitwidth");
        RAISE_ERROR(BADARG_ATOM);
    }
    adc_bitwidth_t bit_width = (adc_bitwidth_t) bits;

    term attenuation = argv[2];
    VALIDATE_VALUE(attenuation, term_is_atom);
    adc_atten_t atten = interop_atom_term_select_int(attenuation_table, attenuation, ctx->global);
    if (UNLIKELY(atten == ADC_INVALID_PARAM)) {
        ESP_LOGE(TAG, "invalid attenuation");
        RAISE_ERROR(BADARG_ATOM);
#ifdef ADC_ATTEN_DB_12  // TODO: Update message to ESP_LOGW after ESP-IDf v5.1 is EOL; then all devices that support db_12 will deprecate db_11.
    } else if ((atten == ADC_ATTEN_DB_11)) {
        ESP_LOGI(TAG, "attenuation 'db_11' replaced by 'db_12' and will be deprecated for devices that support 'db_12' in a future release, devices that support it should use 'db_12' instead.");
        atten = ADC_ATTEN_DB_12;
#endif
    }

    term unit_term = argv[3];

    struct UnitResource *unit_rsrc = NULL;
    if (UNLIKELY(!is_adc_resource(ctx->global, unit_term))) {
        ESP_LOGE(TAG, "handle supplied is not a valid adc resource");
        RAISE_ERROR(BADARG_ATOM);
    }
    if (UNLIKELY(!to_unit_resource(unit_term, &unit_rsrc, ctx))) {
        ESP_LOGE(TAG, "resource supplied is not a valid adc unit resource");
        RAISE_ERROR(BADARG_ATOM);
    }
    // TODO: DO we need to increment unit_rsrc reference counter here?
    // int increment = enif_keep_resource(unit_rsrc);
    // if (UNLIKELY(increment == 0)) {
    //     ESP_LOGW(TAG, "failed to increment unit resource reference count");
    // }
    adc_oneshot_unit_handle_t unit_handle = NULL;
    if (adc_unit == ADC_UNIT_1) {
        unit_handle = unit_rsrc->unit_handle;
    }
#ifdef CONFIG_AVM_ADC2_ENABLE
    else if (adc_unit == ADC_UNIT_2) {
        unit_handle = unit_rsrc->unit2_handle;
    }
#endif
    else {
        ESP_LOGE(TAG, "no enabled ADC unit matching pin");
        RAISE_ERROR(BADARG_ATOM);
    }

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = bit_width,
        .atten = atten,
    };

    err = adc_oneshot_config_channel(unit_handle, adc_channel, &config);
    if (UNLIKELY(err != ESP_OK)) {
        RAISE_ERROR(globalcontext_make_atom(ctx->global, (esp_err_to_name(err))));
    }

    // adc_cali_handle_t cali_handle = NULL;
    struct ChannelResource *chan_rsrc = enif_alloc_resource(adc_channel_resource, sizeof(struct ChannelResource));
    if (IS_NULL_PTR(chan_rsrc)) {
        ESP_LOGE(TAG, "failed to allocate resource: %s:%i.", __FILE__, __LINE__);
    }
    chan_rsrc->cali_handle = NULL;
    cali_method_t calibration = do_adc_calibration(adc_unit, adc_channel, atten, bit_width, &chan_rsrc->cali_handle);

    chan_rsrc->attenuation = atten;
    chan_rsrc->width = bit_width;
    chan_rsrc->adc_unit = adc_unit;
    chan_rsrc->channel = adc_channel;
    chan_rsrc->calibration = calibration;

    if (UNLIKELY(memory_ensure_free(ctx, TERM_BOXED_RESOURCE_SIZE != MEMORY_GC_OK))) {
        enif_release_resource(chan_rsrc);
        ESP_LOGE(TAG, "failed to allocate memory for resource: %s:%i.", __FILE__, __LINE__);
        RAISE_ERROR(OUT_OF_MEMORY_ATOM);
    }

    term chan_obj = enif_make_resource(erl_nif_env_from_context(ctx), chan_rsrc);
    enif_release_resource(chan_rsrc);

    // {ok, {'$adc', resource(), ref()}}
    size_t requested_size = TUPLE_SIZE(2) + TUPLE_SIZE(3) + REF_SIZE;
    ESP_LOGD(TAG, "Requesting memory size %u for return message", requested_size);
    if (UNLIKELY(memory_ensure_free_with_roots(ctx, requested_size, 1, &chan_obj, MEMORY_CAN_SHRINK) != MEMORY_GC_OK)) {
        ESP_LOGE(TAG, "failed to allocate tuple memory size %u: %s:%i.", requested_size, __FILE__, __LINE__);
        RAISE_ERROR(OUT_OF_MEMORY_ATOM);
    }

    term chan_resource = term_alloc_tuple(3, &ctx->heap);
    term_put_tuple_element(chan_resource, 0, globalcontext_make_atom(ctx->global, ATOM_STR("\x4", "$adc")));
    term_put_tuple_element(chan_resource, 1, chan_obj);
    uint64_t ref_ticks = globalcontext_get_ref_ticks(ctx->global);
    term ref = term_from_ref_ticks(ref_ticks, &ctx->heap);
    term_put_tuple_element(chan_resource, 2, ref);

    term ret = term_alloc_tuple(2, &ctx->heap);
    term_put_tuple_element(ret, 0, OK_ATOM);
    term_put_tuple_element(ret, 1, chan_resource);

    return ret;
}

static term nif_adc_release_channel(Context *ctx, int argc, term argv[])
{
    if (UNLIKELY(!is_adc_resource(ctx->global, argv[0]))) {
        ESP_LOGE(TAG, "no valid adc channel resource");
        RAISE_ERROR(BADARG_ATOM);
    }
    term channel_resource = argv[0];
    struct ChannelResource *chan_rsrc;
    if (UNLIKELY(!to_channel_resource(channel_resource, &chan_rsrc, ctx))) {
      ESP_LOGE(TAG, "resource is not a valid adc channel resource");
      RAISE_ERROR(BADARG_ATOM);
    }

    if (chan_rsrc->calibration > ESTIMATED) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        ESP_LOGD(TAG, "deregister curve fitting calibration scheme");
        esp_err_t err = adc_cali_delete_scheme_curve_fitting(chan_rsrc->cali_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "failed to release calibration profile");
            RAISE_ERROR(globalcontext_make_atom(ctx->global, (esp_err_to_name(err))));
        }

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        ESP_LOGD(TAG, "deregister line fitting calibration scheme");
        esp_err_t err = adc_cali_delete_scheme_line_fitting(chan_rsrc->cali_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "failed to release calibration profile");
            RAISE_ERROR(globalcontext_make_atom(ctx->global, (esp_err_to_name(err))));
        }
#endif
    }
    // TODO: (maybe) If unit resource reference counter is incremented in nif_adc_acquire it should be free'd here,
    // if this is the case this function will need to be adc:release_channel/2, and take a unit parameter.

    resource_type_destroy(adc_channel_resource);

    return OK_ATOM;
}

static term nif_adc_sample(Context *ctx, int argc, term argv[])
{
    UNUSED(argc);

    term chan_term = argv[0];
    if (UNLIKELY(!is_adc_resource(ctx->global, chan_term))) {
        ESP_LOGE(TAG, "Invalid channel resource");
        RAISE_ERROR(BADARG_ATOM);
    }
    struct ChannelResource *chan_rsrc;
    if (UNLIKELY(!to_channel_resource(chan_term, &chan_rsrc, ctx))) {
      ESP_LOGE(TAG, "failed to convert adc channel resource");
      RAISE_ERROR(BADARG_ATOM);
    }

    term unit_term = argv[1];
    if (UNLIKELY(!is_adc_resource(ctx->global, unit_term))) {
        ESP_LOGE(TAG, "handle supplied is not a valid adc resource");
        RAISE_ERROR(BADARG_ATOM);
    }
    struct UnitResource *unit_rsrc = NULL;
    if (UNLIKELY(!to_unit_resource(unit_term, &unit_rsrc, ctx))) {
        ESP_LOGE(TAG, "resource supplied is not a valid adc unit resource");
        RAISE_ERROR(BADARG_ATOM);
    }

    adc_oneshot_unit_handle_t unit_handle = NULL;
    if (chan_rsrc->adc_unit == ADC_UNIT_1) {
        unit_handle = unit_rsrc->unit_handle;
#ifdef CONFIG_AVM_ADC2_ENABLE
    } else if (chan_rsrc->adc_unit == ADC_UNIT_2) {
        unit_handle = unit_rsrc->unit2_handle;
#endif
    } else {
        ESP_LOGE(TAG, "no valid unit handle found in resource");
        RAISE_ERROR(BADARG_ATOM);
    }

    term read_options = argv[2];
    VALIDATE_VALUE(read_options, term_is_list);
    term samples = interop_kv_get_value_default(read_options, ATOM_STR("\x7", "samples"), term_from_int32(DEFAULT_SAMPLES), ctx->global);
    int samples_val = term_to_int32(samples);
    ESP_LOGD(TAG, "read samples: %i", samples_val);
    term raw = interop_kv_get_value_default(read_options, ATOM_STR("\x3", "raw"), FALSE_ATOM, ctx->global);
    term voltage = interop_kv_get_value_default(read_options, ATOM_STR("\x7", "voltage"), FALSE_ATOM, ctx->global);

    int adc_reading[samples_val];
    int adc_raw = 0;

    esp_err_t err = ESP_FAIL;
    for (int i = 0; i < samples_val; ++i) {
        err = adc_oneshot_read(unit_handle, chan_rsrc->channel, &adc_reading[i]);
        if (UNLIKELY(err != ESP_OK)) {
            ESP_LOGE(TAG, "adc_oneshot_read read failed for unit: %i channel: %i", (int) unit_handle, (int) chan_rsrc->channel);
            RAISE_ERROR(globalcontext_make_atom(ctx->global, (esp_err_to_name(err))));
        }
        adc_raw += adc_reading[i];
    }

    adc_raw /= samples_val;
    ESP_LOGD(TAG, "read adc raw reading: %i", adc_raw);

    raw = raw == TRUE_ATOM ? term_from_int32(adc_raw) : UNDEFINED_ATOM;
    if (voltage == TRUE_ATOM) {
        int millivolts = 0;
        if (chan_rsrc->calibration > ESTIMATED) {
            err = adc_cali_raw_to_voltage(chan_rsrc->cali_handle, adc_raw, &millivolts);
            if (UNLIKELY(err != ESP_OK)) {
                ESP_LOGW(TAG, "Failed to get calibrated voltage, returning estimated voltage");
                voltage = term_from_int32(approximate_millivolts(adc_raw, chan_rsrc->attenuation, chan_rsrc->width));
            } else {
                voltage = term_from_int32(millivolts);
            }
        } else {
            ESP_LOGD(TAG, "ADC channel not calibrated, using estimated voltage");
            voltage = term_from_int32(approximate_millivolts(adc_raw, chan_rsrc->attenuation, chan_rsrc->width));
        }
    } else {
        voltage = UNDEFINED_ATOM;
    };

    struct SampleValue *measurement = calloc(sizeof(struct SampleValue));
    if(IS_NULL_PTR(measurement)) {
        ESP_LOGE(TAG, "failed to allocate memory: %s:%i.", __FILE__, __LINE__);
        RAISE_ERROR(OUT_OF_MEMORY_ATOM);
    }
    measurement->raw = raw;
    measurement->mv = voltage;

    size_t request_size = TUPLE_SIZE(2) + TUPLE_SIZE(2);
    if (UNLIKELY(memory_ensure_free_with_roots(ctx, request_size, 1, measurement, MEMORY_CAN_SHRINK) != MEMORY_GC_OK)) {
        RAISE_ERROR(OUT_OF_MEMORY_ATOM);
    }
    term values = create_pair(ctx, measurement->raw, measurement->mv);
    free(measurement);
    term ret = create_pair(ctx, OK_ATOM, values);

    return ret;
}

//
// Component Nif Entrypoints
//
static void nif_adc_chan_resource_dtor(ErlNifEnv *caller_env, void *obj)
{
    UNUSED(caller_env);

    struct ChannelResource *chan_rsrc = (struct ChannelResource *) obj;

    if (chan_rsrc->calibration > ESTIMATED) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        ESP_LOGD(TAG, "deregister curve fitting calibration scheme");
        erp_err_t err = adc_cali_delete_scheme_curve_fitting(chan_rsrc->cali_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "failed to release curve fitting calibration profile");
        }

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        ESP_LOGD(TAG, "deregister line fitting calibration scheme");
        esp_err_t err = adc_cali_delete_scheme_line_fitting(chan_rsrc->cali_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "failed to release line fitting calibration profile");
        }
#endif
    }

    resource_type_destroy(adc_channel_resource);
}

static void nif_adc_unit_resource_dtor(ErlNifEnv *caller_env, void *obj)
{
    UNUSED(caller_env);

   struct UnitResource *unit_rsrc = (struct UnitResource *) obj;

    if (!IS_NULL_PTR(unit_rsrc->unit_handle)) {
        esp_err_t err = adc_oneshot_del_unit(unit_rsrc->unit_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "failed to release adc");
        }
    }
#ifdef CONFIG_AVM_ADC2_ENABLE
    if (!IS_NULL_PTR(unit_rsrc->unit2_handle)) {
        esp_err_t err = adc_oneshot_del_unit(unit_rsrc->unit2_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "failed to release adc");
        }
    }
#endif
    resource_type_destroy(adc_unit_resource);
}

static const ErlNifResourceTypeInit ChannelResourceTypeInit = {
    .members = 1,
    .dtor = nif_adc_chan_resource_dtor,
};

static const ErlNifResourceTypeInit UnitResourceTypeInit = {
    .members = 1,
    .dtor = nif_adc_unit_resource_dtor,
};

static const struct Nif adc_init_nif = {
    .base.type = NIFFunctionType,
    .nif_ptr = nif_adc_init
};
static const struct Nif adc_deinit_nif = {
    .base.type = NIFFunctionType,
    .nif_ptr = nif_adc_deinit
};
static const struct Nif adc_acquire_nif = {
    .base.type = NIFFunctionType,
    .nif_ptr = nif_adc_acquire
};
static const struct Nif adc_release_channel_nif = {
    .base.type = NIFFunctionType,
    .nif_ptr = nif_adc_release_channel
};
static const struct Nif adc_sample_nif = {
    .base.type = NIFFunctionType,
    .nif_ptr = nif_adc_sample
};

void atomvm_adc_init(GlobalContext *global)
{
    ErlNifEnv env;
    erl_nif_env_partial_init_from_globalcontext(&env, global);
    adc_channel_resource = enif_init_resource_type(&env, "adc_channel_resource", &ChannelResourceTypeInit, ERL_NIF_RT_CREATE, NULL);
    adc_unit_resource = enif_init_resource_type(&env, "adc_unit_resource", &UnitResourceTypeInit, ERL_NIF_RT_CREATE, NULL);
}

const struct Nif *atomvm_adc_get_nif(const char *nifname)
{
    TRACE("Locating nif %s ...", nifname);
    if (strcmp("adc:init/0", nifname) == 0) {
        TRACE("Resolved platform nif %s ...", nifname);
        return &adc_init_nif;
    }
    if (strcmp("adc:deinit/1", nifname) == 0) {
        TRACE("Resolved platform nif %s ...", nifname);
        return &adc_deinit_nif;
    }
    if (strcmp("adc:acquire/4", nifname) == 0) {
        TRACE("Resolved platform nif %s ...", nifname);
        return &adc_acquire_nif;
    }
    if (strcmp("adc:release_channel/1", nifname) == 0) {
        TRACE("Resolved platform nif %s ...", nifname);
        return &adc_release_channel_nif;
    }
    if (strcmp("adc:sample/3", nifname) == 0) {
        TRACE("Resolved platform nif %s ...", nifname);
        return &adc_sample_nif;
    }
    // TODO: deinit
    return NULL;
}

#include <sdkconfig.h>
#ifdef CONFIG_AVM_ADC_ENABLE
REGISTER_NIF_COLLECTION(atomvm_adc, atomvm_adc_init, NULL, atomvm_adc_get_nif)
#endif
