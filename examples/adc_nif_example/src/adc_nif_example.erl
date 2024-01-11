%%
%% Copyright (c) 2023 <winford@object.stream>
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
-module(adc_nif_example).

-export([start/0]).

start() ->
    Pin = 34,
    {ok, Unit} = adc:init(),
    {ok, Chan} = adc:acquire(Pin, Unit)
    loop(Chan, Unit).

loop(Chan, Unit) ->
    case adc:sample(Chan, Unit) of
        {ok, {Raw, MilliVolts}} ->
            io:format("Raw: ~p Voltage: ~pmV~n", [Raw, MilliVolts]);
        Error ->
            io:format("Error taking reading: ~p~n", [Error])
    end,
    timer:sleep(1000),
    loop(Chan, Unit).
