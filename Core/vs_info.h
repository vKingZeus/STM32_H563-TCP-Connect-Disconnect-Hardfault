#ifndef _VS_INFO_h__
#define _VS_INFO_h__

#define ID_MODEL            "Detector Ethernet"
#define ID_NIG              "107550"

#define ID_MAJOR            "0"
#define ID_MINOR            "98"
#define ID_PATCH            "00"
#define ID_SPECIAL          "00"
#define ID_RC               "RC1"

#define ID_VERSION          "V" ID_MAJOR "." ID_MINOR "." ID_PATCH


#define ID_VERSION_EX       ID_VERSION " - " ID_RC
#define ID_VERSION_SW       ID_VERSION
#define ID_VERSION_FW       ID_NIG " - " ID_MODEL " - " ID_VERSION " - " ID_RC
#define ID_VERSION_HW       "Proto\r\n"
#define ACK                 "ACK!\r\n"
#define NACK                "NACK!\r\n"
#endif

