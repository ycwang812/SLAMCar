/* DO NOT EDIT THIS FILE - it is machine generated */
#include <jni.h>
/* Header for class org_appplay_platformsdk_PlatformSDKNatives */

#ifndef _Included_org_appplay_platformsdk_PlatformSDKNatives
#define _Included_org_appplay_platformsdk_PlatformSDKNatives
#ifdef __cplusplus
extern "C" {
#endif
/*
 * Class:     org_appplay_platformsdk_PlatformSDKNatives
 * Method:    SetPlatformSDK
 * Signature: (Ljava/lang/String;)V
 */
JNIEXPORT void JNICALL Java_org_appplay_platformsdk_PlatformSDKNatives_SetPlatformSDK
  (JNIEnv *, jclass, jstring);

/*
 * Class:     org_appplay_platformsdk_PlatformSDKNatives
 * Method:    OnLoginSuc
 * Signature: (ILjava/lang/String;Ljava/lang/String;Ljava/lang/String;)V
 */
JNIEXPORT void JNICALL Java_org_appplay_platformsdk_PlatformSDKNatives_OnLoginSuc
  (JNIEnv *, jclass, jint, jstring, jstring, jstring);

/*
 * Class:     org_appplay_platformsdk_PlatformSDKNatives
 * Method:    OnLoginCancel
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_org_appplay_platformsdk_PlatformSDKNatives_OnLoginCancel
  (JNIEnv *, jclass);

/*
 * Class:     org_appplay_platformsdk_PlatformSDKNatives
 * Method:    OnLoginFailed
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_org_appplay_platformsdk_PlatformSDKNatives_OnLoginFailed
  (JNIEnv *, jclass);

/*
 * Class:     org_appplay_platformsdk_PlatformSDKNatives
 * Method:    OnGuestOfficialSuc
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_org_appplay_platformsdk_PlatformSDKNatives_OnGuestOfficialSuc
  (JNIEnv *, jclass);

/*
 * Class:     org_appplay_platformsdk_PlatformSDKNatives
 * Method:    OnPaySuc
 * Signature: (Ljava/lang/String;Z)V
 */
JNIEXPORT void JNICALL Java_org_appplay_platformsdk_PlatformSDKNatives_OnPaySuc
  (JNIEnv *, jclass, jstring, jboolean);

/*
 * Class:     org_appplay_platformsdk_PlatformSDKNatives
 * Method:    OnPayCancel
 * Signature: (Ljava/lang/String;Z)V
 */
JNIEXPORT void JNICALL Java_org_appplay_platformsdk_PlatformSDKNatives_OnPayCancel
  (JNIEnv *, jclass, jstring, jboolean);

/*
 * Class:     org_appplay_platformsdk_PlatformSDKNatives
 * Method:    OnPayFailed
 * Signature: (Ljava/lang/String;Z)V
 */
JNIEXPORT void JNICALL Java_org_appplay_platformsdk_PlatformSDKNatives_OnPayFailed
  (JNIEnv *, jclass, jstring, jboolean);

/*
 * Class:     org_appplay_platformsdk_PlatformSDKNatives
 * Method:    OnPaySMSSent
 * Signature: (Ljava/lang/String;Z)V
 */
JNIEXPORT void JNICALL Java_org_appplay_platformsdk_PlatformSDKNatives_OnPaySMSSent
  (JNIEnv *, jclass, jstring, jboolean);

/*
 * Class:     org_appplay_platformsdk_PlatformSDKNatives
 * Method:    OnPayRequestSubmitted
 * Signature: (Ljava/lang/String;Z)V
 */
JNIEXPORT void JNICALL Java_org_appplay_platformsdk_PlatformSDKNatives_OnPayRequestSubmitted
  (JNIEnv *, jclass, jstring, jboolean);

/*
 * Class:     org_appplay_platformsdk_PlatformSDKNatives
 * Method:    OnPayError
 * Signature: (Ljava/lang/String;IZ)V
 */
JNIEXPORT void JNICALL Java_org_appplay_platformsdk_PlatformSDKNatives_OnPayError
  (JNIEnv *, jclass, jstring, jint, jboolean);

#ifdef __cplusplus
}
#endif
#endif
