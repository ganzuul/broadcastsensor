/****************************************************************************
**
** Copyright (C) 2011 Nokia Corporation and/or its subsidiary(-ies).
** All rights reserved.
** Contact: Nokia Corporation (qt-info@nokia.com)
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** You may use this file under the terms of the BSD license as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of Nokia Corporation and its Subsidiary(-ies) nor
**     the names of its contributors may be used to endorse or promote
**     products derived from this software without specific prior written
**     permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
** $QT_END_LICENSE$
**
****************************************************************************/

#include <QtGui>
#include <QtNetwork>

#include "sender.h"
#include "qaccelerometer.h"

Sender::Sender(QWidget *parent)
    : QWidget(parent)
{
    statusLabel = new QLabel(tr("Ready to broadcast datagrams on port 45454"));
    statusLabel->setWordWrap(true);

    startButton = new QPushButton(tr("&Start"));
    quitButton = new QPushButton(tr("&Quit"));

    buttonBox = new QDialogButtonBox;
    buttonBox->addButton(startButton, QDialogButtonBox::ActionRole);
    buttonBox->addButton(quitButton, QDialogButtonBox::RejectRole);

    timer = new QTimer(this);
//! [0]
    udpSocket = new QUdpSocket(this);
//! [0]
    messageNoX = 1;
    messageNoY = 1;
    messageNoZ = 1;

    connect(startButton, SIGNAL(clicked()), this, SLOT(startBroadcasting()));
    connect(quitButton, SIGNAL(clicked()), this, SLOT(close()));
//    connect(timer, SIGNAL(timeout()), this, SLOT(broadcastDatagram()));

    QVBoxLayout *mainLayout = new QVBoxLayout;
    mainLayout->addWidget(statusLabel);
    mainLayout->addWidget(buttonBox);
    setLayout(mainLayout);

    setWindowTitle(tr("Broadcast Sender"));
}

class smoothedaccelerometerfilter : public QObject, public QAccelerometerFilter
{
    qreal prevX;
    qreal prevY;
    qreal prevZ;
    bool havePrev;

public:
    smoothedaccelerometerfilter(QObject *parent = 0)
        : QObject(parent)
        , QAccelerometerFilter()
        , prevX(0)
        , prevY(0)
        , prevZ(0)
        , havePrev(false)
    {
    }

    bool filter(QAccelerometerReading *reading)
    {
        // Smooth out the reported values.  Large changes are applied as-is,
        // and small jitters smooth to the rest position.
        if (havePrev) {
            qreal xdiff = reading->x() - prevX;
            qreal ydiff = reading->y() - prevY;
            qreal zdiff = reading->z() - prevZ;
#define threshold 0.196133f
            if (qAbs(xdiff) < threshold && qAbs(ydiff) < threshold && qAbs(zdiff) < threshold) {
                reading->setX(prevX + xdiff * 0.1f);
                reading->setY(prevY + ydiff * 0.1f);
                reading->setZ(prevZ + zdiff * 0.1f);
            }
        }
        prevX = reading->x();
        prevY = reading->y();
        prevZ = reading->z();
        havePrev = true;
        return true;
    }
};


void Sender::startBroadcasting()
{
    startButton->setEnabled(false);
    sensor = new QAccelerometer(this);
    connect(sensor, SIGNAL(readingChanged()), this, SLOT(accelerometerTimeout()));
    sensor->addFilter(new smoothedaccelerometerfilter(this));
    sensor->start();
//    timer->start(1000);
}

void Sender::accelerometerTimeout()
{
    messageNoX = sensor->reading()->x();
    messageNoY = sensor->reading()->y();
    messageNoZ = sensor->reading()->z();
    broadcastDatagram();
}

#define ACCEL_TO_G(v) (v / 9.80665)

/*
QVector3D Sender::gravity() const
{
    qreal x = ACCEL_TO_G(sensor->reading()->x()) * sensitivity;
    qreal y = ACCEL_TO_G(sensor->reading()->y()) * sensitivity;
    qreal z = ACCEL_TO_G(sensor->reading()->z());

    return QVector3D(x, y, z);
}
*/

void Sender::broadcastDatagram()
{
    statusLabel->setText(tr("Now broadcasting datagram %1, %2, %3").arg(messageNoX).arg(messageNoY).arg(messageNoZ));
//! [1]
    QByteArray datagram = QByteArray::number(messageNoX) +
            ", "+ QByteArray::number(messageNoY) +", "+ QByteArray::number(messageNoZ);
    udpSocket->writeDatagram(datagram.data(), datagram.size(),
                             QHostAddress::Broadcast, 45454);
//! [1]
}
