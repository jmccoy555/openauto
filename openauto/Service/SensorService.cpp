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

#include <chrono>
#include <cmath>

#include "aasdk_proto/DrivingStatusEnum.pb.h"
#include "OpenautoLog.hpp"
#include "openauto/Service/SensorService.hpp"

namespace openauto
{
namespace service
{

SensorService::SensorService(boost::asio::io_service& ioService, aasdk::messenger::IMessenger::Pointer messenger, bool nightMode)
    : strand_(ioService)
    , channel_(std::make_shared<aasdk::channel::sensor::SensorServiceChannel>(strand_, std::move(messenger)))
    , nightMode_(nightMode)
    , locationTimer_(ioService)
{

}

void SensorService::start()
{
    strand_.dispatch([this, self = this->shared_from_this()]() {
        OPENAUTO_LOG(info) << "[SensorService] start.";
        channel_->receive(this->shared_from_this());
    });
}

void SensorService::stop()
{
    strand_.dispatch([this, self = this->shared_from_this()]() {
        OPENAUTO_LOG(info) << "[SensorService] stop.";
        locationStarted_ = false;
        carSpeedStarted_ = false;
        locationTimer_.cancel();
    });
}

void SensorService::fillFeatures(aasdk::proto::messages::ServiceDiscoveryResponse& response)
{
    OPENAUTO_LOG(info) << "[SensorService] fill features.";

    auto* channelDescriptor = response.add_channels();
    channelDescriptor->set_channel_id(static_cast<uint32_t>(channel_->getId()));
    auto* sensorChannel = channelDescriptor->mutable_sensor_channel();
    sensorChannel->add_sensors()->set_type(aasdk::proto::enums::SensorType::DRIVING_STATUS);
    sensorChannel->add_sensors()->set_type(aasdk::proto::enums::SensorType::LOCATION);
    sensorChannel->add_sensors()->set_type(aasdk::proto::enums::SensorType::CAR_SPEED);
    sensorChannel->add_sensors()->set_type(aasdk::proto::enums::SensorType::NIGHT_DATA);
}

void SensorService::onChannelOpenRequest(const aasdk::proto::messages::ChannelOpenRequest& request)
{
    OPENAUTO_LOG(info) << "[SensorService] open request, priority: " << request.priority();
    const aasdk::proto::enums::Status::Enum status = aasdk::proto::enums::Status::OK;
    OPENAUTO_LOG(info) << "[SensorService] open status: " << status;

    aasdk::proto::messages::ChannelOpenResponse response;
    response.set_status(status);

    auto promise = aasdk::channel::SendPromise::defer(strand_);
    promise->then([]() {}, std::bind(&SensorService::onChannelError, this->shared_from_this(), std::placeholders::_1));
    channel_->sendChannelOpenResponse(response, std::move(promise));

    channel_->receive(this->shared_from_this());
}

void SensorService::onSensorStartRequest(const aasdk::proto::messages::SensorStartRequestMessage& request)
{
    OPENAUTO_LOG(info) << "[SensorService] sensor start request, type: " << request.sensor_type();

    aasdk::proto::messages::SensorStartResponseMessage response;
    response.set_status(aasdk::proto::enums::Status::OK);

    auto promise = aasdk::channel::SendPromise::defer(strand_);

    if(request.sensor_type() == aasdk::proto::enums::SensorType::DRIVING_STATUS)
    {
        promise->then(std::bind(&SensorService::sendDrivingStatusUnrestricted, this->shared_from_this()),
                      std::bind(&SensorService::onChannelError, this->shared_from_this(), std::placeholders::_1));
    }
    else if(request.sensor_type() == aasdk::proto::enums::SensorType::NIGHT_DATA)
    {
        promise->then(std::bind(&SensorService::sendNightData, this->shared_from_this()),
                      std::bind(&SensorService::onChannelError, this->shared_from_this(), std::placeholders::_1));
    }
    else if(request.sensor_type() == aasdk::proto::enums::SensorType::LOCATION)
    {
        promise->then(std::bind(&SensorService::scheduleLocationUpdate, this->shared_from_this()),
                      std::bind(&SensorService::onChannelError, this->shared_from_this(), std::placeholders::_1));
    }
    else if(request.sensor_type() == aasdk::proto::enums::SensorType::CAR_SPEED)
    {
        // Shares scheduleLocationUpdate()/sendLocationData()'s single timer
        // rather than starting a second one - see carSpeedStarted_'s comment.
        carSpeedStarted_ = true;
        promise->then(std::bind(&SensorService::scheduleLocationUpdate, this->shared_from_this()),
                      std::bind(&SensorService::onChannelError, this->shared_from_this(), std::placeholders::_1));
    }
    else
    {
        promise->then([]() {}, std::bind(&SensorService::onChannelError, this->shared_from_this(), std::placeholders::_1));
    }

    channel_->sendSensorStartResponse(response, std::move(promise));
    channel_->receive(this->shared_from_this());
}

void SensorService::sendDrivingStatusUnrestricted()
{
    aasdk::proto::messages::SensorEventIndication indication;
    indication.add_driving_status()->set_status(aasdk::proto::enums::DrivingStatus::UNRESTRICTED);

    auto promise = aasdk::channel::SendPromise::defer(strand_);
    promise->then([]() {}, std::bind(&SensorService::onChannelError, this->shared_from_this(), std::placeholders::_1));
    channel_->sendSensorEventIndication(indication, std::move(promise));
}

void SensorService::sendNightData()
{
    aasdk::proto::messages::SensorEventIndication indication;
    indication.add_night_mode()->set_is_night(nightMode_);

    auto promise = aasdk::channel::SendPromise::defer(strand_);
    promise->then([]() {}, std::bind(&SensorService::onChannelError, this->shared_from_this(), std::placeholders::_1));
    channel_->sendSensorEventIndication(indication, std::move(promise));
}

void SensorService::onChannelError(const aasdk::error::Error& e)
{
    OPENAUTO_LOG(error) << "[SensorService] channel error: " << e.what();
}

void SensorService::setNightMode(bool nightMode)
{
    nightMode_ = nightMode;
    this->sendNightData();
}

void SensorService::setLocation(double latitude, double longitude, double altitude, double speed, double bearing, double accuracy)
{
    strand_.dispatch([this, self = this->shared_from_this(), latitude, longitude, altitude, speed, bearing, accuracy]() {
        hasLocation_ = true;
        latitude_ = latitude;
        longitude_ = longitude;
        altitude_ = altitude;
        speed_ = speed;
        bearing_ = bearing;
        accuracy_ = accuracy;
    });
}

// Kicks off on the phone's first SensorStartRequest for LOCATION and keeps
// rescheduling itself roughly once a second for as long as the sensor
// stays started - stop() cancels locationTimer_ to end the chain rather
// than this checking some other "still open" flag each time.
void SensorService::scheduleLocationUpdate()
{
    locationStarted_ = true;
    this->sendLocationData();

    locationTimer_.expires_after(std::chrono::seconds(1));
    locationTimer_.async_wait(strand_.wrap([this, self = this->shared_from_this()](const boost::system::error_code& ec) {
        if (!ec && locationStarted_)
            this->scheduleLocationUpdate();
    }));
}

void SensorService::sendLocationData()
{
    // Nothing latched via setLocation() yet - skip rather than send a
    // fabricated 0,0 fix while waiting on a real one.
    if (!hasLocation_)
        return;

    auto now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());

    aasdk::proto::messages::SensorEventIndication indication;

    // Gated independently, not just on hasLocation_ - a phone could in
    // principle start one of these without the other, and sending gps_
    // location/speed for a sensor it never asked for would be a real
    // (if likely harmless) protocol impurity.
    if (locationStarted_) {
        auto* location = indication.add_gps_location();
        location->set_timestamp(static_cast<uint64_t>(now.count()));
        // GPSLocation's latitude/longitude are fixed-point int32 - degrees x1e7,
        // Android's own standard convention for this (fits int32's range with
        // ~1cm precision at the equator). Confirmed working live - position
        // tracks correctly on the phone with this encoding.
        location->set_latitude(static_cast<google::protobuf::int32>(std::lround(latitude_ * 1e7)));
        location->set_longitude(static_cast<google::protobuf::int32>(std::lround(longitude_ * 1e7)));
        location->set_accuracy(static_cast<google::protobuf::uint32>(std::max(0.0, accuracy_)));
        location->set_altitude(static_cast<google::protobuf::int32>(std::lround(altitude_)));
        location->set_speed(static_cast<google::protobuf::int32>(std::lround(speed_)));
        location->set_bearing(static_cast<google::protobuf::int32>(std::lround(bearing_)));
    }

    if (carSpeedStarted_) {
        // Confirmed live: Waze showed no speed reading at all with only
        // GPSLocation.speed set (position worked fine) - switching back to
        // phone GPS made speed work again, meaning Waze's on-screen speed
        // reads from this separate CAR_SPEED sensor, not from the location
        // fix's own embedded speed field. Same m/s value either way (both
        // mirror Android's plain-meters-per-second vehicle speed
        // convention) - just offered through both channels now since
        // there's no way to know which any given nav app actually prefers.
        indication.add_speed()->set_speed(static_cast<google::protobuf::int32>(std::lround(speed_)));
    }

    auto promise = aasdk::channel::SendPromise::defer(strand_);
    promise->then([]() {}, std::bind(&SensorService::onChannelError, this->shared_from_this(), std::placeholders::_1));
    channel_->sendSensorEventIndication(indication, std::move(promise));
}

}
}
