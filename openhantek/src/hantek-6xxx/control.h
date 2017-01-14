////////////////////////////////////////////////////////////////////////////////
//
//  OpenHantek
/// \file hantek-6xxx/control.h
/// \brief Declares the Hantek6xxx::Control class.
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


#ifndef HANTEK_6XXX_CONTROL_H
#define HANTEK_6XXX_CONTROL_H

#include "dsocontrol.h"

extern "C" {
#include <libusb-1.0/libusb.h>
}

namespace Hantek6xxx {
	class Device;

	//////////////////////////////////////////////////////////////////////////////
	/// \class Control                                            hantek-6xxx/control.h
	/// \brief Implementation of the DsoControl abstraction layer for %Hantek 6xxx USB DSOs.
	class Control : public DsoControl {
		Q_OBJECT

		public:
			Control(QObject *parent = 0);
			~Control();

			unsigned int getChannelCount() override;
			QList<unsigned int> *getAvailableRecordLengths() override;
			double getMinSamplerate() override;
			double getMaxSamplerate() override;

		protected:
			void run() override;

			// Results
			std::vector<std::vector<double> > samples; ///< Sample data vectors sent to the data analyzer
			unsigned int previousSampleCount; ///< The expected total number of samples at the last check before sampling started
			QMutex samplesMutex; ///< Mutex for the sample data

		public slots:
			void connectDevice() override;

			void startSampling() override;
			void stopSampling() override;

			unsigned int setRecordLength(unsigned int size) override;
			double setSamplerate(double samplerate = 0.0) override;
			double setRecordTime(double duration = 0.0) override;

			int setChannelUsed(unsigned int channel, bool used) override;
			int setCoupling(unsigned int channel, Dso::Coupling coupling) override;
			double setGain(unsigned int channel, double gain) override;
			double setOffset(unsigned int channel, double offset) override;

			int setTriggerMode(Dso::TriggerMode mode) override;
			int setTriggerSource(bool special, unsigned int id) override;
			double setTriggerLevel(unsigned int channel, double level) override;
			int setTriggerSlope(Dso::Slope slope) override;
			double setPretriggerPosition(double position) override;
			int forceTrigger() override;

	#ifdef DEBUG
			int stringCommand(QString command) override;
	#endif
		private:
			libusb_context *usbContext;
			libusb_device_handle *device;

			QList<unsigned int> availableRecordLengths;
	};

} // namespace Hantek6xxx
#endif // HANTEK_6XXX_CONTROL_H
