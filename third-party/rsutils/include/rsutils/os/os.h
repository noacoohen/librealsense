// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2023 Intel Corporation. All Rights Reserved.

#pragma once
#include <string>

namespace rsutils
{
    namespace os
    {
        enum special_folder
        {
            user_desktop,
            user_documents,
            user_pictures,
            user_videos,
            temp_folder,
            app_data
        };
        std::string get_os_name();
        std::string get_platform_name();
        std::string get_folder_path(special_folder f);
    }
}
