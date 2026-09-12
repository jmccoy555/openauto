/*
*  This file is part of openauto project.
*  Copyright (C) 2018 f1x.studio (Michal Szwaj)
*
*  openauto is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation; either version 3 of the License, or
*  (at your option) any later version.

*  openauto is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with openauto. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <boost/asio/steady_timer.hpp>

#include "aasdk/Channel/Sensor/SensorServiceChannel.hpp"
#include "IService.hpp"

namespace openauto
{
namespace service
{

class SensorService: public aasdk::channel::sensor::ISensorServiceChannelEventHandler, public IService, public std::enable_shared_from_this<SensorService>
{
public:
    SensorService(boost::asio::io_service& ioService, aasdk::messenger::IMessenger::Pointer messenger, bool nightMode=false);

    void start() override;
    void stop() override;
    void fillFeatures(aasdk::proto::messages::ServiceDiscoveryResponse& response) override;
    void onChannelOpenRequest(const aasdk::proto::messages::ChannelOpenRequest& request) override;
    void onSensorStartRequest(const aasdk::proto::messages::SensorStartRequestMessage& request) override;
    void onChannelError(const aasdk::error::Error& e) override;
    void setNightMode(bool nightMode);

    // Units follow Android's own Location getters, since that's what's on
    // the other end of this - degrees for lat/lon/bearing, metres for
    // altitude/accuracy, metres/second for speed. Just latches the values;
    // actually sending happens on scheduleLocationUpdate()'s own ~1s cadence
    // once the phone has asked for this sensor, not on every call here.
    void setLocation(double latitude, double longitude, double altitude, double speed, double bearing, double accuracy);

private:
    using std::enable_shared_from_this<SensorService>::shared_from_this;
    void sendDrivingStatusUnrestricted();
    void sendNightData();
    void sendLocationData();
    void scheduleLocationUpdate();

    boost::asio::io_service::strand strand_;
    aasdk::channel::sensor::SensorServiceChannel::Pointer channel_;
    bool nightMode_;

    boost::asio::steady_timer locationTimer_;
    bool timerRunning_ = false;  // guards against LOCATION and CAR_SPEED each kicking off their own copy of the same recurring chain - see scheduleLocationUpdate()
    bool locationStarted_ = false;
    // CAR_SPEED is a distinct SensorType from LOCATION (see SensorTypeEnum.proto)
    // - some nav apps' own on-screen speed readout apparently sources from
    // this dedicated sensor rather than GPSLocation's embedded speed field
    // (confirmed live: Waze showed no speed at all with LOCATION-only
    // injection, but worked once switched to phone GPS - phone-sourced
    // location comes with the OS's own speed handling attached, which our
    // injected LOCATION alone doesn't replace). Piggybacks on the same
    // scheduleLocationUpdate()/sendLocationData() cycle rather than a
    // second timer, since both sensors share the same underlying value.
    bool carSpeedStarted_ = false;
    bool hasLocation_ = false;  // nothing latched via setLocation() yet - scheduleLocationUpdate() keeps retrying but sendLocationData() skips sending until this is true
    double latitude_ = 0;
    double longitude_ = 0;
    double altitude_ = 0;
    double speed_ = 0;
    double bearing_ = 0;
    double accuracy_ = 0;
};

}
}
