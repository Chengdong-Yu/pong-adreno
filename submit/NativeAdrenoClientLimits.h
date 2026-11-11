/* SPDX-License-Identifier: MIT */
#pragma once
// The wallpaper gallery legitimately opens more than sixteen native Metal
// devices. Queue identities and isolated process page tables share this cap.
enum : unsigned {NativeAdrenoMaxClients=32};
