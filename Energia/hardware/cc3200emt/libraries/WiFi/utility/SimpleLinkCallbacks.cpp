/*
 SimpleLinkCallbacks.c
 Author: Noah Luskey | LuskeyNoah@gmail.com
 Use: Place code for callbacks in this file if desired.
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */


extern "C" {
    #include "simplelink.h"
    #include <string.h>
}

#include "WiFi.h"
#include <ti/runtime/wiring/Energia.h>

#if (defined(sl_GeneralEvtHdlr))
extern void sl_GeneralEvtHdlr(SlDeviceEvent_t *pSlDeviceEvent)
{
    /* TODO: this function is called on SL_OPCODE_DEVICE_DEVICEASYNCFATALERROR
     *       and SL_OPCODE_DEVICE_ABORT events.  Sould probably not ignore
     *       these events.
     */
}
#endif

//
// WLAN Event Handler: Modifies static variable in WiFi
// to indicate if a connection to an AP is made or not
//
#if (defined(sl_WlanEvtHdlr))
extern void sl_WlanEvtHdlr(SlWlanEvent_t *pSlWlanEvent)
{
    unsigned char tmp[4] = {0,0,0,0};

    switch (pSlWlanEvent->Event) {
        
        //
        //Wlan has connected to a station
        //brackets necessary to avoid crosses initialization error
        //
        case SL_WLAN_CONNECT_EVENT: {
            WiFiClass::WiFi_status = WL_CONNECTED;
            //
            //copy ssid name to WiFiClass and manually add null terminator
            //
            char* pSSID = (char*)pSlWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_name;
            uint8_t ssidLength = pSlWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_len;
            if (ssidLength > MAX_SSID_LEN) {
                return;
            }
            memcpy(WiFiClass::connected_ssid, pSSID, ssidLength);
            WiFiClass::connected_ssid[ssidLength] = '\0';
            
            //
            //copy bssid to WiFiClass (no null terminator. Length always = 6)
            //
            char* pBSSID = (char*)pSlWlanEvent->EventData.STAandP2PModeWlanConnected.bssid;
            WiFiClass::connected_bssid[5] = pBSSID[0];
            WiFiClass::connected_bssid[4] = pBSSID[1];
            WiFiClass::connected_bssid[3] = pBSSID[2];
            WiFiClass::connected_bssid[2] = pBSSID[3];
            WiFiClass::connected_bssid[1] = pBSSID[4];
            WiFiClass::connected_bssid[0] = pBSSID[5];
            break;
        }
            
        //
        //Wlan has disconnected, so completely zero out the ssid and bssid
        //
        case SL_WLAN_DISCONNECT_EVENT:
            WiFiClass::WiFi_status = WL_DISCONNECTED;
            memset(WiFiClass::connected_ssid, 0, MAX_SSID_LEN);
            memset(WiFiClass::connected_bssid, 0, BSSID_LEN);
            
            break;

        /* Track station connects & disconnects in AP mode */
        case SL_WLAN_STA_CONNECTED_EVENT:
            /* Register the MAC w/o an IP; later on, when an IP is leased to this user, the _latestConnect index will update
             * to point to this one (inside sl_NetAppEvtHdlr's run of _registerNewDeviceIP() below).
             */
            WiFiClass::_registerNewDeviceIP(tmp, pSlWlanEvent->EventData.APModeStaConnected.mac);
            break;

        case SL_WLAN_STA_DISCONNECTED_EVENT:
            WiFiClass::_unregisterDevice(pSlWlanEvent->EventData.APModestaDisconnected.mac);
            WiFiClass::_connectedDeviceCount--;
            break;
            
        default:
            break;
    }
}
#endif

#if (defined(sl_NetAppEvtHdlr))
extern void sl_NetAppEvtHdlr(SlNetAppEvent_t *pSlSockEvent)
{
    switch (pSlSockEvent->Event) {
        //
        //IP address acquired. Copy the uint32 to the WiFiClass static variable
        //do the following for both IPV4 and IPV6
        //
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
        case SL_NETAPP_IPV6_IPACQUIRED_EVENT:
        {
            WiFiClass::local_IP = pSlSockEvent->EventData.ipAcquiredV4.ip;
            break;
        }

        /* Track station IP leases in AP mode */
        case SL_NETAPP_IP_LEASED_EVENT:
            unsigned char ipAddrAry[4];

            ipAddrAry[0] = (pSlSockEvent->EventData.ipLeased.ip_address >> 24);
            ipAddrAry[1] = (pSlSockEvent->EventData.ipLeased.ip_address >> 16) & 0xFF;
            ipAddrAry[2] = (pSlSockEvent->EventData.ipLeased.ip_address >> 8) & 0xFF;
            ipAddrAry[3] = pSlSockEvent->EventData.ipLeased.ip_address & 0xFF;
            WiFiClass::_registerNewDeviceIP(ipAddrAry, pSlSockEvent->EventData.ipLeased.mac);
            WiFiClass::_connectedDeviceCount++;
            break;

        default:
            break;
    }
}
#endif

#if (defined(sl_SockEvtHdlr))
extern void sl_SockEvtHdlr(SlSockEvent_t *pSlSockEvent)
{
}
#endif

#if (defined(sl_HttpServerCallback))
extern void sl_HttpServerCallback(SlHttpServerEvent_t *pSlHttpServerEvent, SlHttpServerResponse_t *pSlHttpServerResponse)
{
}
#endif
