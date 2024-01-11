%%
%% Copyright (c) 2020-2023 dushin.net
%% Copyright (c) 2022-2024 Winford <winford@object.stream>
%% All rights reserved.
%%
%% Licensed under the Apache License, Version 2.0 (the "License");
%% you may not use this file except in compliance with the License.
%% You may obtain a copy of the License at
%%
%%     http://www.apache.org/licenses/LICENSE-2.0
%%
%% Unless required by applicable law or agreed to in writing, software
%% distributed under the License is distributed on an "AS IS" BASIS,
%% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
%% See the License for the specific language governing permissions and
%% limitations under the License.
%%
%%-----------------------------------------------------------------------------
%% @doc ADC support.
%%
%% Use this module to take ADC readings. On an ESP32 device ADC unit1 allows
%% taking reading from pins 32-39. ADC unit2 allows pins 0, 2, 4, 12-15, and
%% 25-27 to be used as long as WiFi is not required by the application. The pins
%% available for ADC use vary by device, check your datasheet for specific
%% hardware support, and possible pin conflicts when WiFi is used simultaneously
%% with ADC unit2.
%% @end
%%-----------------------------------------------------------------------------
-module(adc).

%% Low level resource based nif functions
-export([ acquire/4, init/0, release_channel/1, deinit/1, sample/3 ]).
%% Nif convenience functions
-export([ acquire/2, sample/2 ]).

-type adc_rsrc() :: {'$adc', Resource::binary(), Ref::reference()}.
-type adc_pin() ::  non_neg_integer().
%% ADC enabled pins vary by chipset. Consult your datasheet.
-type bit_width() :: bit_9 | bit_10 | bit_11 | bit_12 | bit_13 | bit_max.
%% The default `bit_max' will select the highest value supported by the chipset. Note: `bit_11' is deprecated and
%% will be removed in the future, behaves the same as `bit_12'.
-type attenuation() :: db_0 | db_2_5 | db_6 | db_11 | db_12.
%% The decibel gain determines the maximum save voltage to be measured. Consult the datasheet for your device to
%% determine the voltage ranges supported by each gain setting.
-type read_options() :: [read_option()].
-type read_option() :: raw | voltage | {samples, non_neg_integer()}.

-type raw_value() :: 0..511|1023|2047|4095|8191 | undefined.
%% The maximum analog value is determined by bit_width().
-type voltage_reading() :: 0..3300 | undefined.
%% The maximum safe millivolt value to measure is determined by attenuation().
-type reading() :: {raw_value() | undefined, voltage_reading() | undefined}.

-define(ADC_RSRC, {'$adc', _Resource, _Ref}).

-define(DEFAULT_SAMPLES, 64).
-define(DEFAULT_READ_OPTIONS, [raw, voltage, {samples, ?DEFAULT_SAMPLES}]).

%%-----------------------------------------------------------------------------
%% @returns {ok, ADCUnit :: adc_rsrc()} | {error, Reason}
%% @doc     Initialize the ADC unit hardware.
%%
%% The returned ADC unit resource must be supplied for all future ADC operations.
%% @end
%%-----------------------------------------------------------------------------
-spec init() -> {ok, ADCUnit :: adc_rsrc()} | {error, Reason::term()}.
init() ->
    throw(nif_error).

%%-----------------------------------------------------------------------------
%% @param   UnitResource returned from init/0
%% @returns ok | {error, Reason}
%% @doc     Nif to release the adc unit resource returned from init/0.
%%
%% Stop the ADC driver and free the unit resource. ADC channels should be released
%% using `release_channel/1' to free each configured channel before freeing the
%% unit resource.
%% @end
%%-----------------------------------------------------------------------------
-spec deinit(UnitResource :: adc_rsrc()) -> ok | {error, Reason :: term()}.
deinit(_UnitResource) ->
    throw(nif_error).

%%-----------------------------------------------------------------------------
%% @param   Pin         Pin to configure as ADC
%% @param   UnitHandle  The unit handle returned from `init/0'
%% @equiv   acquire(Pin, bit_max, db_11, UnitHandle)
%% @returns {ok, Channel::adc_rsrc()} | {error, Reason}
%% @doc     Initialize an ADC pin.
-spec acquire(Pin :: adc_pin(), UnitHandle :: adc_rsrc()) -> {ok, Channel :: adc_rsrc()} | {error, Reason::term()}.
acquire(_Pin, _UnitHandle) ->
    throw(nif_error).

%%-----------------------------------------------------------------------------
%% @param   Pin         Pin to configure as ADC
%% @param   BitWidth    Resolution in bit to measure
%% @param   Attenuation Decibel gain for voltage range
%% @param   UnitHandle  The unit handle returned from `init/0'
%% @returns {ok, Channel::adc_rsrc()} | {error, Reason}
%% @doc     Initialize an ADC pin.
%%
%% The BitWidth value `bit_max' may be used to automatically select the highest
%% sample rate supported by your ESP chip-set.
%%
%% The Attenuation value can be used to adust the gain, and therefore safe
%% measurement range on voltage the exact range of voltages supported by each
%% db gain varies by chip, the chart below shows the general ranges supported,
%% consult the data sheet for exact range of your model. The gain of `db_12' is
%% only supported on some models, beginning with ESP-IDF v5.3 the setting of
%% `db_11' is deprecated, but currently will fallback to the `db_12' option. This
%% option will be removed after ESP-IDF v5.2 reaches end of support, and users are
%% urged to update their application to use `db_12' for builds with v5.3 or later.
%%
%% %% <table>
%%   <tr> <th>Attenuation</th><th>Min Millivolts</th><th>Max Millivolts</th></tr>
%%   <tr> <td>`db_0'</td> <td>0-100</td> <td>750-950</td></tr>
%%   <tr> <td>`db_2_5'</td> <td>0-100</td> <td>1050-1250</td></tr>
%%   <tr> <td>`db_6'</td> <td>0-150</td> <td>1300-1750</td></tr>
%%   <tr> <td>`bd_11' | `db_12'</td> <td>0-150</td> <td>2450-2500</td></tr>
%% </table> 
%%
%% Use the returned `Channel' reference in subsequent ADC operations on
%% the same pin.
%% @end
%%-----------------------------------------------------------------------------
-spec acquire(Pin :: adc_pin(), BitWidth :: bit_width(), Attenuation :: attenuation(), UnitHandle :: adc_rsrc()) -> {ok, Channel :: adc_rsrc()} | {error, Reason::term()}.
acquire(_Pin, _BitWidth, _Attenuation, _UnitHandle) ->
    throw(nif_error).

%%-----------------------------------------------------------------------------
%% @param   ChannelResource of the pin returned from acquire/4
%% @returns ok | {error, Reason}
%% @doc     Deinitialize the specified ADC channel.
%%
%% @end
%%-----------------------------------------------------------------------------
-spec release_channel(ChannelResource :: adc_rsrc()) -> ok | {error, Reason :: term()}.
release_channel(_ChannelResource) ->
    throw(nif_error).

%%-----------------------------------------------------------------------------
%% @param   ChannelResource of the pin returned from acquire/4
%% @param   UnitResource of the pin returned from init/0
%% @returns {ok, {RawValue, MilliVolts}} | {error, Reason}
%% @equiv   sample(ChannelResource, UnitResource, [raw, voltage, {samples, 64}])
%% @doc     Take a reading using default values from the pin associated with this ADC.
%%
%% @end
%%-----------------------------------------------------------------------------
-spec sample(ChannelResource :: adc_rsrc(), UnitResource :: adc_rsrc()) -> {ok, reading()} | {error, Reason :: term()}.
sample(ChannelResource, UnitResource) ->
    ?MODULE:sample(ChannelResource, UnitResource, ?DEFAULT_READ_OPTIONS).

%%-----------------------------------------------------------------------------
%% @param   ChannelResource of the pin returned from acquire/4
%% @param   UnitResource of the pin returned from init/0
%% @param   ReadOptions extra options
%% @returns {ok, {RawValue, MilliVolts}} | {error, Reason}
%% @doc     Take a reading from the pin associated with this ADC.
%%
%% The Options parameter may be used to specify the behavior of the read
%% operation.
%%
%% If the ReadOptions contains the atom `raw', then the raw value will be returned
%% in the first element of the returned tuple.  Otherwise, this element will be the
%% atom `undefined'.
%%
%% If the ReadOptions contains the atom `voltage', then the voltage value will be returned
%% in millivolts in the second element of the returned tuple.  Otherwise, this element will
%% be the atom `undefined'.
%%
%% You may specify the number of samples to be taken and averaged over using the tuple
%% `{samples, Samples::pos_integer()}'.
%%
%% If the error `Reason' is timeout and the adc channel is on unit 2 then WiFi is likely
%% enabled and adc2 readings will no longer be possible.
%% @end
%%-----------------------------------------------------------------------------
-spec sample(ChannelResource :: adc_rsrc(), UnitResource :: adc_rsrc(), ReadOptions :: read_options()) -> {ok, Result :: reading()} | {error, Reason :: term()}.
sample(_ChannelResource, _UnitResource, _ReadOptions) ->
    throw(nif_error).
