#ifdef __APPLE__

#import <Foundation/Foundation.h>
#include "AppNap.h"

// Held for the process lifetime so the "latency-critical" activity stays in effect
// (App Nap resumes the moment the activity token is released).
static id<NSObject> gAppNapToken = nil;

void disableAppNap()
{
    if (gAppNapToken != nil)
        return;   // already active for this process

    NSActivityOptions opts = NSActivityUserInitiated | NSActivityLatencyCritical;
    gAppNapToken = [[NSProcessInfo processInfo]
        beginActivityWithOptions:opts
                          reason:@"ephys-socket real-time UDP acquisition"];

   #if !__has_feature(objc_arc)
    [gAppNapToken retain];   // beginActivityWithOptions: returns an autoreleased object
   #endif

    NSLog(@"[ephys-socket] App Nap disabled for acquisition (NSActivityLatencyCritical)");
}

#endif  // __APPLE__
