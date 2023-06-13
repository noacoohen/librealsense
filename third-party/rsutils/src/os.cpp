// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2023 Intel Corporation. All Rights Reserved.

#include "rsutils/os/os.h"
#include <third-party/glfw/include/GLFW/glfw3.h>

#ifdef _WIN32
#include <windows.h>
#include <wchar.h>
#include <KnownFolders.h>
#include <shlobj.h>
#include <rsutils/os/hresult.h>
#endif

#if (defined(_WIN32) || defined(_WIN64))
#include "ShellAPI.h"
#endif

#if defined __linux__ || defined __APPLE__
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#endif



namespace rsutils
{
    namespace os 
    {
        std::string get_os_name()
        {
            #ifdef _WIN32
            return "Windows";
            #else
            #ifdef __APPLE__
            return "Mac OS";
            #else
            #ifdef __linux__
            return "Linux";
            #else
            return "Unknown";
            #endif
            #endif
            #endif
        }

        std::string get_platform_name()
        {
            #ifdef _WIN64
            return "Windows amd64";
            #elif _WIN32
            return "Windows x86";
            #elif __linux__
            #ifdef __arm__
            return "Linux arm";
            #else
            return "Linux amd64";
            #endif
            #elif __APPLE__
            return "Mac OS";
            #elif __ANDROID__
            return "Linux arm";
            #else
            return "";
            #endif

        }

        std::string get_folder_path(special_folder f)
        {
            std::string res;
            #ifdef _WIN32

            if ( f == temp_folder )
            {
                TCHAR buf[MAX_PATH];
                if ( GetTempPath(MAX_PATH, buf) != 0 )
                {
                    char str[1024];
                    wcstombs(str, buf, 1023);
                    res = str;
                }
            }
            else
            {
                GUID folder;
                HRESULT hr;
                switch ( f )
                {
                case user_desktop: folder = FOLDERID_Desktop;
                    break;
                case user_documents: folder = FOLDERID_Documents;
                    // The user's Documents folder location may get overridden, as we know OneDrive does in certain circumstances.
                    // In such cases, the new function, SHGetKnownFolderPath, does not always return the new path, while the deprecated
                    // function does.
                    CHAR path[MAX_PATH];
                    CHECK_HR(SHGetFolderPathA(NULL, CSIDL_PERSONAL, NULL, 0, path));

                    res = path;
                    res += "\\";
                    return res;
                case user_pictures: folder = FOLDERID_Pictures;
                    break;
                case user_videos: folder = FOLDERID_Videos;
                    break;
                case app_data: folder = FOLDERID_RoamingAppData;
                    break;
                default:
                    throw std::invalid_argument(
                        std::string("Value of f (") + std::to_string(f) + std::string(") is not supported"));
                }

                PWSTR folder_path = NULL;
                hr = SHGetKnownFolderPath(folder, KF_FLAG_DEFAULT_PATH, NULL, &folder_path);
                if ( SUCCEEDED(hr) )
                {
                    char str[1024];
                    wcstombs(str, folder_path, 1023);
                    CoTaskMemFree(folder_path);
                    res = str;
                    res += "\\";
                }
                else
                {
                    throw std::runtime_error("Failed to get requested special folder");
                }
            }
            #endif //_WIN32
            #if defined __linux__ || defined __APPLE__
            if ( f == special_folder::temp_folder )
            {
                const char * tmp_dir = getenv("TMPDIR");
                res = tmp_dir ? tmp_dir : "/tmp/";
            }
            else
            {
                const char * home_dir = getenv("HOME");
                if ( !home_dir )
                {
                    struct passwd * pw = getpwuid(getuid());
                    home_dir = ( pw && pw->pw_dir ) ? pw->pw_dir : "";
                }
                if ( home_dir )
                {
                    res = home_dir;
                    switch ( f )
                    {
                    case user_desktop: res += "/Desktop/";
                        break;
                    case user_documents: res += "/Documents/";
                        break;
                    case user_pictures: res += "/Pictures/";
                        break;
                    case user_videos: res += "/Videos/";
                        break;
                    case app_data: res += "/.";
                        break;
                    default:
                        throw std::invalid_argument(
                            std::string("Value of f (") + std::to_string(f) + std::string(") is not supported"));
                    }
                }
            }
            #endif // defined __linux__ || defined __APPLE__
            return res;
        }
    }  
}

