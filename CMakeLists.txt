#
# This file is part of AtomVM.
#
# Copyright 2022 Fred Dushin <fred@dushin.net>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0 OR LGPL-2.1-or-later
#
if (IDF_VERSION_MAJOR EQUAL 4)
    FATAL_ERROR("ESP-IDF versions prior to v 5.0 not supported")
else()
    set(ATOMVM_ADC_COMPONENT_SRCS
        "nifs/atomvm_adc.c"
    )
    set(ADC_COMPONENTS "esp_adc")
endif()

idf_component_register(
    SRCS ${ATOMVM_ADC_COMPONENT_SRCS}
    INCLUDE_DIRS "nifs/include"
    PRIV_REQUIRES "libatomvm" "avm_sys" ${ADC_COMPONENTS}
)

idf_build_set_property(
    LINK_OPTIONS "-Wl,--whole-archive;${CMAKE_CURRENT_BINARY_DIR}/lib${COMPONENT_NAME}.a;-Wl,--no-whole-archive"
    APPEND
)
