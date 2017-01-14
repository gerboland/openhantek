////////////////////////////////////////////////////////////////////////////////
//
//  OpenHantek
//  hantek-6xxx/control.cpp
//
//  Copyright (C) 2008, 2009  Oleg Khudyakov
//  prcoder@potrebitel.ru
//  Copyright (C) 2010 - 2012  Oliver Haag
//  oliver.haag@gmail.com
//  Copyright (C) 2017  Gerry Boland
//  g@gerryboland.net
//
//  This program is free software: you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation, either version 3 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along with
//  this program.  If not, see <http://www.gnu.org/licenses/>.
//
////////////////////////////////////////////////////////////////////////////////


#include <cmath>
#include <limits>

#include <QList>
#include <QMutex>
#include <QDebug>

#include "hantek-6xxx/control.h"

#include "helper.h"

#define HANTEK_CHANNELS		2

#define SAMPLERATE_VALUES \
	SR_MHZ(48), SR_MHZ(30), SR_MHZ(24), \
	SR_MHZ(16), SR_MHZ(8), SR_MHZ(4), \
	SR_MHZ(1), SR_KHZ(500), SR_KHZ(200), \
	SR_KHZ(100),

#define SAMPLERATE_REGS \
	48, 30, 24, 16, 8, 4, 1, 50, 20, 10,

#define VDIV_VALUES \
	{ 100, 1000 }, \
	{ 250, 1000 }, \
	{ 500, 1000 }, \
	{ 1, 1 },

#define VDIV_REG \
	10, 5, 2, 1,

#define VDIV_MULTIPLIER		10
/* Weird flushing needed for filtering glitch away. */
#define FLUSH_PACKET_SIZE	1024

#define MIN_PACKET_SIZE		512
#define MAX_PACKET_SIZE		(12 * 1024 * 1024)

#define HANTEK_EP_IN		0x86

namespace Hantek6xxx {
	enum control_requests {
		VDIV_CH1_REG   = 0xe0,
		VDIV_CH2_REG   = 0xe1,
		SAMPLERATE_REG = 0xe2,
		TRIGGER_REG    = 0xe3,
		CHANNELS_REG   = 0xe4,
		COUPLING_REG   = 0xe5,
	};

	/// \brief Initializes the command buffers and lists.
	/// \param parent The parent widget.
	Control::Control(QObject *parent)
		: DsoControl(parent)
		, usbContext(NULL)
		, device(NULL) {
		this->previousSampleCount = 0;

//		connect(this->device, SIGNAL(disconnected()), this, SLOT(disconnectDevice()));
	}

	/// \brief Disconnects the device.
	Control::~Control() {
		libusb_release_interface(this->device, 0);
		libusb_close(this->device);
		libusb_exit(this->usbContext);
	}

	/// \brief Try to connect to the oscilloscope.
	void Control::connectDevice() {
		int ret = libusb_init(&this->usbContext);
		if (ret < 0) {
			QString message = QString("Unable to init libusb: %1").arg(libusb_error_name(ret));
			qDebug() << message;
			emit statusMessage(message, 0);
		}

		libusb_set_debug(this->usbContext, 3);

		this->device = libusb_open_device_with_vid_pid(this->usbContext, 0x1D50, 0x608E);
		if (!this->device) {
			QString message("Unable to open device");
			qDebug() << message;
			emit statusMessage(message, 0);
			return;
		}

		if (libusb_kernel_driver_active(this->device, 0) == 1) {
			if (libusb_detach_kernel_driver(this->device, 0) != 0) {
				libusb_close(this->device);
				this->device = nullptr;

				QString message = QString("Unable to detach kernel to use: %1").arg(libusb_error_name(ret));
				qDebug() << message;
				emit statusMessage(message, 0);
				return;
			}
		}

		if (libusb_claim_interface(this->device, 0) < 0) {
			libusb_close(this->device);
			this->device = nullptr;

			QString message = QString("Unable to claim interface: %1").arg(libusb_error_name(ret));
			qDebug() << message;
			emit statusMessage(message, 0);
			return;
		}

		// Emit signals for initial settings
//		emit availableRecordLengthsChanged(this->settings.samplerate.limits->recordLengths);
//		updateSamplerateLimits();
//		emit recordLengthChanged(this->settings.samplerate.limits->recordLengths[this->settings.recordLengthId]);
//		if(this->settings.samplerate.limits->recordLengths[this->settings.recordLengthId] != UINT_MAX)
//			emit recordTimeChanged((double) this->settings.samplerate.limits->recordLengths[this->settings.recordLengthId] / this->settings.samplerate.current);
//		emit samplerateChanged(this->settings.samplerate.current);

		DsoControl::connectDevice();
	}

	/// \brief Gets the physical channel count for this oscilloscope.
	/// \return The number of physical channels.
	unsigned int Control::getChannelCount() {
		return HANTEK_CHANNELS;
	}

	/// \brief Start sampling process.
	void Control::startSampling() {
		if(!this->device)
			return;

		uchar value = 0x01;
		int ret = libusb_control_transfer(this->device, LIBUSB_REQUEST_TYPE_VENDOR, TRIGGER_REG, 0x00, 0x00, &value, 1, 100);
		if (ret < 0) {
			qDebug("startSampling failed with error: %s", libusb_error_name(ret));
		} else {
			qDebug("startSampling returned %d (number of bytes transferred successfully)", ret);
		}

		DsoControl::startSampling();
	}

	/// \brief Stop sampling process.
	void Control::stopSampling() {
		if(!this->device)
			return;

		uchar value = 0x00;
		int ret = libusb_control_transfer(this->device, LIBUSB_REQUEST_TYPE_VENDOR, TRIGGER_REG, 0x00, 0x00, &value, 1, 100);
		if (ret < 0) {
			qDebug("stopSampling failed with error: %s", libusb_error_name(ret));
		} else {
			qDebug("stopSampling returned %d (number of bytes transferred successfully)", ret);
		}

		DsoControl::stopSampling();
	}

	/// \brief Get available record lengths for this oscilloscope.
	/// \return The number of physical channels, empty list for continuous.
	QList<unsigned int> *Control::getAvailableRecordLengths() {
		// IMPLEMENT
		return &this->availableRecordLengths;
	}

	/// \brief Get minimum samplerate for this oscilloscope.
	/// \return The minimum samplerate for the current configuration in S/s.
	double Control::getMinSamplerate() {
		return 100000; // 100Khz
	}

	/// \brief Get maximum samplerate for this oscilloscope.
	/// \return The maximum samplerate for the current configuration in S/s.
	double Control::getMaxSamplerate() {
		return 48000000; // 48Mhz
	}

	/// \brief Handles all USB things until the device gets disconnected.
	void Control::run() {
		// Initialize communication thread state

		// The control loop is running until the device is disconnected
		exec();

		emit statusMessage(tr("The device has been disconnected"), 0);
	}
/*
	/// \brief Gets sample data from the oscilloscope and converts it.
	/// \return sample count on success, libusb error code on error.
	int Control::getSamples(bool process) {
		if (this->device->getModel() != MODEL_DSO6022BE) {
			// Request data
			errorCode = this->device->bulkCommand(this->command[BULK_GETDATA], 1);
			if(errorCode < 0)
				return errorCode;
		}

		// Save raw data to temporary buffer
		bool fastRate = false;
		unsigned int totalSampleCount = this->getSampleCount(&fastRate);
		if(totalSampleCount == UINT_MAX)
			return LIBUSB_ERROR_INVALID_PARAM;

		// To make sure no samples will remain in the scope buffer, also check the sample count before the last sampling started
		if(totalSampleCount < this->previousSampleCount) {
			unsigned int currentSampleCount = totalSampleCount;
			totalSampleCount = this->previousSampleCount;
			this->previousSampleCount = currentSampleCount; // Using sampleCount as temporary buffer since it was set to totalSampleCount
		}
		else {
			this->previousSampleCount = totalSampleCount;
		}

		unsigned int sampleCount = totalSampleCount;
		if(!fastRate)
			sampleCount /= HANTEK_CHANNELS;
		unsigned int dataLength = totalSampleCount;
		if(this->specification.sampleSize > 8)
			dataLength *= 2;

		unsigned char data[dataLength];
		errorCode = this->device->bulkReadMulti(data, dataLength);
		if(errorCode < 0)
			return errorCode;

		// Process the data only if we want it
		if(process) {
			// How much data did we really receive?
			dataLength = errorCode;
			if(this->specification.sampleSize > 8)
				totalSampleCount = dataLength / 2;
			else
				totalSampleCount = dataLength;

			this->samplesMutex.lock();

			// Convert channel data
			if(fastRate) {
				// Fast rate mode, one channel is using all buffers
				sampleCount = totalSampleCount;
				int channel = 0;
				for(; channel < HANTEK_CHANNELS; ++channel) {
					if(this->settings.voltage[channel].used)
						break;
				}

				// Clear unused channels
				for(int channelCounter = 0; channelCounter < HANTEK_CHANNELS; ++channelCounter)
					if(channelCounter != channel) {

						this->samples[channelCounter].clear();
					}

				if(channel < HANTEK_CHANNELS) {
					// Resize sample vector
					this->samples[channel].resize(sampleCount);

					// Convert data from the oscilloscope and write it into the sample buffer
					unsigned int bufferPosition = this->settings.trigger.point * 2;
					if(this->specification.sampleSize > 8) {
						// Additional most significant bits after the normal data
						unsigned int extraBitsPosition; // Track the position of the extra bits in the additional byte
						unsigned int extraBitsSize = this->specification.sampleSize - 8; // Number of extra bits
						unsigned short int extraBitsMask = (0x00ff << extraBitsSize) & 0xff00; // Mask for extra bits extraction

						for(unsigned int realPosition = 0; realPosition < sampleCount; ++realPosition, ++bufferPosition) {
							if(bufferPosition >= sampleCount)
								bufferPosition %= sampleCount;

							extraBitsPosition = bufferPosition % HANTEK_CHANNELS;

							this->samples[channel][realPosition] = ((double) ((unsigned short int) data[bufferPosition] + (((unsigned short int) data[sampleCount + bufferPosition - extraBitsPosition] << (8 - (HANTEK_CHANNELS - 1 - extraBitsPosition) * extraBitsSize)) & extraBitsMask)) / this->specification.voltageLimit[channel][this->settings.voltage[channel].gain] - this->settings.voltage[channel].offsetReal) * this->specification.gainSteps[this->settings.voltage[channel].gain];
						}
					}
					else {
						for(unsigned int realPosition = 0; realPosition < sampleCount; ++realPosition, ++bufferPosition) {
							if(bufferPosition >= sampleCount)
								bufferPosition %= sampleCount;

							this->samples[channel][realPosition] = ((double) data[bufferPosition] / this->specification.voltageLimit[channel][this->settings.voltage[channel].gain] - this->settings.voltage[channel].offsetReal) * this->specification.gainSteps[this->settings.voltage[channel].gain];
						}
					}
				}
			}
			else {
				// Normal mode, channels are using their separate buffers
				sampleCount = totalSampleCount / HANTEK_CHANNELS;
				for(int channel = 0; channel < HANTEK_CHANNELS; ++channel) {
					if(this->settings.voltage[channel].used) {
						// Resize sample vector
						this->samples[channel].resize(sampleCount);

						// Convert data from the oscilloscope and write it into the sample buffer
						unsigned int bufferPosition = this->settings.trigger.point * 2;
						if(this->specification.sampleSize > 8) {
							// Additional most significant bits after the normal data
							unsigned int extraBitsSize = this->specification.sampleSize - 8; // Number of extra bits
							unsigned short int extraBitsMask = (0x00ff << extraBitsSize) & 0xff00; // Mask for extra bits extraction
							unsigned int extraBitsIndex = 8 - channel * 2; // Bit position offset for extra bits extraction

							for(unsigned int realPosition = 0; realPosition < sampleCount; ++realPosition, bufferPosition += HANTEK_CHANNELS) {
								if(bufferPosition >= totalSampleCount)
									bufferPosition %= totalSampleCount;

								this->samples[channel][realPosition] = ((double) ((unsigned short int) data[bufferPosition + HANTEK_CHANNELS - 1 - channel] + (((unsigned short int) data[totalSampleCount + bufferPosition] << extraBitsIndex) & extraBitsMask)) / this->specification.voltageLimit[channel][this->settings.voltage[channel].gain] - this->settings.voltage[channel].offsetReal) * this->specification.gainSteps[this->settings.voltage[channel].gain];
							}
						}
						else {
							if (this->device->getModel() == MODEL_DSO6022BE)
								bufferPosition += channel;
							else
								bufferPosition += HANTEK_CHANNELS - 1 - channel;

							for(unsigned int realPosition = 0; realPosition < sampleCount; ++realPosition, bufferPosition += HANTEK_CHANNELS) {
								if(bufferPosition >= totalSampleCount)
									bufferPosition %= totalSampleCount;

								if (this->device->getModel() == MODEL_DSO6022BE)
									this->samples[channel][realPosition] = (((double) data[bufferPosition] - 0x83)
										 / this->specification.voltageLimit[channel][this->settings.voltage[channel].gain])
										* this->specification.gainSteps[this->settings.voltage[channel].gain];
								else
									this->samples[channel][realPosition] = ((double) data[bufferPosition]
										 / this->specification.voltageLimit[channel][this->settings.voltage[channel].gain]
										 - this->settings.voltage[channel].offsetReal)
										* this->specification.gainSteps[this->settings.voltage[channel].gain];
							}
						}
					}
					else {
						// Clear unused channels
						this->samples[channel].clear();
					}
				}
			}

			this->samplesMutex.unlock();
#ifdef DEBUG
			static unsigned int id = 0;
			++id;
			Helper::timestampDebug(QString("Received packet %1").arg(id));
#endif
			emit samplesAvailable(&(this->samples), this->settings.samplerate.current, this->settings.samplerate.limits->recordLengths[this->settings.recordLengthId] == UINT_MAX, &(this->samplesMutex));
		}

		return errorCode;
	}


*/

	/// \brief Sets the size of the oscilloscopes sample buffer.
	/// \param index The record length index that should be set.
	/// \return The record length that has been set, 0 on error.
	unsigned int Control::setRecordLength(unsigned int index) {
		if(!this->device)
			return 0;

		// IMPLEMENT
		return 1;
	}

	/// \brief Sets the samplerate of the oscilloscope.
	/// \param samplerate The samplerate that should be met (S/s), 0.0 to restore current samplerate.
	/// \return The samplerate that has been set, 0.0 on error.
	double Control::setSamplerate(double samplerate) {
		if(!this->device)
			return 0.0;

		// IMPLEMENT
		return 1.0;
	}

	/// \brief Sets the time duration of one aquisition by adapting the samplerate.
	/// \param duration The record time duration that should be met (s), 0.0 to restore current record time.
	/// \return The record time duration that has been set, 0.0 on error.
	double Control::setRecordTime(double duration) {
		if(!this->device)
			return 0.0;

		// IMPLEMENT
		return 1.0;
	}

	/// \brief Enables/disables filtering of the given channel.
	/// \param channel The channel that should be set.
	/// \param used true if the channel should be sampled.
	/// \return See ::Dso::ErrorCode.
	int Control::setChannelUsed(unsigned int channel, bool used) {
		if(!this->device)
			return Dso::ERROR_CONNECTION;

		if(channel >= HANTEK_CHANNELS)
			return Dso::ERROR_PARAMETER;

		// IMPLEMENT
		return Dso::ERROR_NONE;
	}

	/// \brief Set the coupling for the given channel.
	/// \param channel The channel that should be set.
	/// \param coupling The new coupling for the channel.
	/// \return See ::Dso::ErrorCode.
	int Control::setCoupling(unsigned int channel, Dso::Coupling coupling) {
		if(!this->device)
			return Dso::ERROR_CONNECTION;

		if(channel >= HANTEK_CHANNELS)
			return Dso::ERROR_PARAMETER;

	//	if (this->device->getModel() == MODEL_DSO6022BE)
	//		Dso::ERROR_NONE;

		// IMPLEMENT

		return Dso::ERROR_NONE;
	}

	/// \brief Sets the gain for the given channel.
	/// \param channel The channel that should be set.
	/// \param gain The gain that should be met (V/div).
	/// \return The gain that has been set, ::Dso::ErrorCode on error.
	double Control::setGain(unsigned int channel, double gain) {
		if(!this->device)
			return Dso::ERROR_CONNECTION;

		if(channel >= HANTEK_CHANNELS)
			return Dso::ERROR_PARAMETER;

		// IMPLEMENT
		return 1.0;
	}

	/// \brief Set the offset for the given channel.
	/// \param channel The channel that should be set.
	/// \param offset The new offset value (0.0 - 1.0).
	/// \return The offset that has been set, ::Dso::ErrorCode on error.
	double Control::setOffset(unsigned int channel, double offset) {
		if(!this->device)
			return Dso::ERROR_CONNECTION;

		if(channel >= HANTEK_CHANNELS)
			return Dso::ERROR_PARAMETER;

		// IMPLEMENT
		return 1.0;
	}

	/// \brief Set the trigger mode.
	/// \return See ::Dso::ErrorCode.
	int Control::setTriggerMode(Dso::TriggerMode mode) {
		if(!this->device)
			return Dso::ERROR_CONNECTION;

		if(mode < Dso::TRIGGERMODE_AUTO || mode > Dso::TRIGGERMODE_SINGLE)
			return Dso::ERROR_PARAMETER;

		// IMPLEMENT
		return Dso::ERROR_NONE;
	}

	/// \brief Set the trigger source.
	/// \param special true for a special channel (EXT, ...) as trigger source.
	/// \param id The number of the channel, that should be used as trigger.
	/// \return See ::Dso::ErrorCode.
	int Control::setTriggerSource(bool special, unsigned int id) {
		if(!this->device)
			return Dso::ERROR_CONNECTION;

		if((!special && id >= HANTEK_CHANNELS))
			return Dso::ERROR_PARAMETER;

		// IMPLEMENT

		return Dso::ERROR_NONE;
	}

	/// \brief Set the trigger level.
	/// \param channel The channel that should be set.
	/// \param level The new trigger level (V).
	/// \return The trigger level that has been set, ::Dso::ErrorCode on error.
	double Control::setTriggerLevel(unsigned int channel, double level) {
		if(!this->device)
			return Dso::ERROR_CONNECTION;

		if(channel >= HANTEK_CHANNELS)
			return Dso::ERROR_PARAMETER;

//		if (this->device->getModel() == MODEL_DSO6022BE)
//			return Dso::ERROR_NONE;

		// IMPLEMENT
		return 1.0;
	}

	/// \brief Set the trigger slope.
	/// \param slope The Slope that should cause a trigger.
	/// \return See ::Dso::ErrorCode.
	int Control::setTriggerSlope(Dso::Slope slope) {
		if(!this->device)
			return Dso::ERROR_CONNECTION;

		if(slope != Dso::SLOPE_NEGATIVE && slope != Dso::SLOPE_POSITIVE)
			return Dso::ERROR_PARAMETER;

		// IMPLEMENT
		return Dso::ERROR_NONE;
	}

	int Control::forceTrigger() {
		// IMPLEMENT
		return 0;
	}

	/// \brief Set the trigger position.
	/// \param position The new trigger position (in s).
	/// \return The trigger position that has been set.
	double Control::setPretriggerPosition(double position) {
		if(!this->device)
			return -2;
		// IMPLEMENT
		return 0.0;
	}

#ifdef DEBUG
	/// \brief Sends bulk/control commands directly.
	/// <p>
	///		<b>Syntax:</b><br />
	///		<br />
	///		Bulk command:
	///		<pre>send bulk [<em>hex data</em>]</pre>
	///		%Control command:
	///		<pre>send control [<em>hex code</em>] [<em>hex data</em>]</pre>
	/// </p>
	/// \param command The command as string (Has to be parsed).
	/// \return See ::Dso::ErrorCode.
	int Control::stringCommand(QString command) {
		if(!this->device)
			return Dso::ERROR_CONNECTION;

		return Dso::ERROR_UNSUPPORTED;
	}
#endif

} // namespace Hantek6xxx
