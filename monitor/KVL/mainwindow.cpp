/*
 *  mainwindow.cpp
 *
 *  Copyright (c) 2013 Nathaniel Lewis, Robotics Society at UC Merced
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QImage>
#include <QMouseEvent>
#include <QToolTip>
#include <QPainter>
#include <QMessageBox>
#include <sstream>

#include <eigen3/Eigen/Eigen>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    // Setup the UI
    ui->setupUi(this);

    // Flags
    drawing = false;
    valid = false;
    n = false;

    // Create the image grabber thread
    downloaderThread = new QThread;

    // Create the image download manager object
    downloader = new frame_downloader("kybernetes", 8080);
    downloader->moveToThread(downloaderThread);
    downloader->mode = false;

    // Attach started and frame grabbed signals to the main window
    connect(downloaderThread, SIGNAL(started()), downloader, SLOT(run()));
    connect(downloader, SIGNAL(frame_grabbed(uint,uint,uint,void*)), this, SLOT(receive_frame(uint,uint,uint,void*)));

    // Setup the capture button
    window = new ControlsWindow(this);
    window->show();
    connect(window, SIGNAL(capture_frame()), this, SLOT(begin_capture()));
    connect(window, SIGNAL(result_option_changed(int)), this, SLOT(changeMode(int)));
    connect(window, SIGNAL(reset_ptz()), this, SLOT(resetPtz()));
    connect(window, SIGNAL(tracking_option_changed(int)), this, SLOT(changeTrackingMode(int)));
}

MainWindow::~MainWindow()
{
    delete downloader;
    delete downloaderThread;

    delete ui;
}

void MainWindow::receive_frame(unsigned int width, unsigned int height, unsigned int size, void *image_buffer)
{
    // Perform resize of the window adjusting to incoming image size
    resize(width, height);
    ui->centralWidget->resize(width, height);
    ui->imageLabel->resize(width, height);

    // The image to store our content in (creates the destination buffer)
    QImage  image(QSize(width, height), QImage::Format_ARGB32);

    // Check if the mode is color image or grayscale track resultant
    if(!downloader->mode)
    {
        // Color image
        // Get pointers to the input and output buffers
        unsigned char *input = (unsigned char *) image_buffer;
        unsigned char *output = (unsigned char *) image.scanLine(0);

        // YUV to RGB matrix
        Eigen::Matrix<float, 3, 3> conversionMatrix;
        conversionMatrix << 1.0, 0.0, 1.28033, 1, -0.21482, -0.38059, 1, 2.12798, 0;

        // Perform conversion
        for(unsigned int i = 0; i < width * height * 2; i += 4)
        {
            // Input matricies
            Eigen::Matrix<float, 3, 1> inputMatrix, inputMatrix2;
            inputMatrix  << ((float)input[0]) / 255.0, ((float)input[1] - 128.0) / 255.0, ((float)input[3] - 128.0) / 255.0;
            inputMatrix2 << ((float)input[2]) / 255.0, ((float)input[1] - 128.0) / 255.0, ((float)input[3] - 128.0) / 255.0;

            // Run conversions
            Eigen::Matrix<float, 3, 1> outputMatrix = conversionMatrix * inputMatrix * 255.0;
            Eigen::Matrix<float, 3, 1> outputMatrix2 = conversionMatrix * inputMatrix2 * 255.0;

            // Store results
            output[0] = (outputMatrix(2,0) < 255) ? ((outputMatrix(2,0) > 0) ? outputMatrix(2,0) : 0) : 255;
            output[1] = (outputMatrix(1,0) < 255) ? ((outputMatrix(1,0) > 0) ? outputMatrix(1,0) : 0) : 255;
            output[2] = (outputMatrix(0,0) < 255) ? ((outputMatrix(0,0) > 0) ? outputMatrix(0,0) : 0) : 255;
            output[3] = 255;
            output[4] = (outputMatrix2(2,0) < 255) ? ((outputMatrix2(2,0) > 0) ? outputMatrix2(2,0) : 0) : 255;
            output[5] = (outputMatrix2(1,0) < 255) ? ((outputMatrix2(1,0) > 0) ? outputMatrix2(1,0) : 0) : 255;
            output[6] = (outputMatrix2(0,0) < 255) ? ((outputMatrix2(0,0) > 0) ? outputMatrix2(0,0) : 0) : 255;
            output[7] = 255;

            // Progress pointers
            input += 4;
            output += 8;
        }

        // If the coordinates are valid we need to calculate the mean of the color range
        if(valid && n)
        {
            // Calculate the corners of the rectangle we need the mean of
            QPoint uL((press.x() < release.x()) ? press.x() : release.x(), (press.y() < release.y()) ? press.y() : release.y());
            QPoint lR((press.x() > release.x()) ? press.x() : release.x(), (press.y() > release.y()) ? press.y() : release.y());
            QPoint lL(uL.x(), lR.y());
            QPoint uR(lR.x(), uL.y());

            // Reset the input pointer to the image buffer
            input = (unsigned char *) image_buffer;

            // Variables to store the mean
            unsigned int count = 0;
            double       meanY = 0.0;
            double       meanU = 0.0;
            double       meanV = 0.0;

            // Sum the pixel components
            for(int j = uL.y(); j < lR.y(); j++)
            {
                // Row element scan
                for(int i = uL.x(); i < lR.x(); i+=2)
                {
                    unsigned int base = (j * 2 * width) + ((i / 2) * 4);
                    meanY += input[base + 0];
                    meanU += input[base + 1];
                    meanY += input[base + 2];
                    meanV += input[base + 3];
                    count++;
                }
            }

            // Divide the means by contained components
            meanY /= (count * 2);
            meanU /= count;
            meanV /= count;

            // Generate a string
            std::ostringstream out;
            out << "Means = Y:" << round(meanY);
            out << ", U:" << round(meanU);
            out << ", V:" << round(meanV);
            out << std::ends;

            // Display message
            QMessageBox message(this);
            message.setText(out.str().c_str());
            message.setInformativeText("Upload with padding to Kybernetes?");
            message.setStandardButtons(QMessageBox::Ok | QMessageBox::Save);
            message.setDefaultButton(QMessageBox::Ok);

            // Figure out what to do with the result
            switch(message.exec())
            {
                // If we hit save, upload to kybernetes
                case QMessageBox::Save:
                    downloader->modify_tracking_parameters(meanY, meanU, meanV);
                    break;

                // If we just hit okay, ignore saving
                case QMessageBox::Ok:
                default:
                    break;
            }
            n = false;
        }
    } else {
        // Get pointers to the input and output buffer
        unsigned char *input = (unsigned char *) image_buffer;
        unsigned char *output = (unsigned char *) image.scanLine(0);

        // Iterate through and set each color component to the input, because this
        // is a grayscale input
        for(unsigned int i = 0; i < width * height; i++)
        {
            // Store results
            output[0] = *input;
            output[1] = *input;
            output[2] = *input;
            output[3] = 255;

            // Progress pointers
            input++;
            output += 4;
        }
    }

    // Display the converted image
    pixmapMutex.lock();
    pixmap = QPixmap::fromImage(image);
    pixmapMutex.unlock();

    // Call to update the image
    updateImage();

    // Free the received buffer 83 119 99
    free(image_buffer);
    //downloaderThread->start();
}

void MainWindow::begin_capture()
{
    // When the fetch image button is pressed, begin to capture the image
    downloaderThread->start();
}

void MainWindow::mousePressEvent(QMouseEvent *event)
{
    // If the left button was the one not pressed, invalidate the track (clear it)
    if(event->button() != Qt::LeftButton)
    {
        valid = false;
        updateImage();
        return;
    }

    // Store the mouse down location (with respect to the pixmap)
    press = release = QPoint(event->pos().x(), event->pos().y());

    // Only set the valid flag if the mouse was released within the bounds of the
    valid = (release.x() >= 0 && release.x() < ui->imageLabel->width()
             && release.y() >= 0 && release.y() < ui->imageLabel->height()) ? true : false;

    // Set the drawing flag and clear the valid flag
    drawing = true;

    // Update image
    updateImage();
}

void MainWindow::mouseReleaseEvent(QMouseEvent *event)
{
    // check which button
    if(event->button() != Qt::LeftButton)
    {
        valid = false;
        updateImage();
        return;
    }

    // Store the mouse up location (with respect to the pixmap)
    release = QPoint(event->pos().x(), event->pos().y());

    // Only set the valid flag if the mouse was released within the bounds of the
    valid = (release.x() >= 0 && release.x() < ui->imageLabel->width()
             && release.y() >= 0 && release.y() < ui->imageLabel->height()) ? true : false;

    // Clear the drawing flag
    drawing = false;
    n = true;

    // Update image
    updateImage();
}

void MainWindow::mouseMoveEvent(QMouseEvent *event)
{   
    // Unless we are not drawing ignore
    if(!drawing) return;

    // Store the mouse position with respect to the pixmap
    release = QPoint(event->pos().x(), event->pos().y());

    // Check if the mouse position is valid to our window
    valid = (release.x() >= 0 && release.x() < ui->imageLabel->width()
             && release.y() >= 0 && release.y() < ui->imageLabel->height()) ? true : false;

    // Update the image
    updateImage();
}

void MainWindow::updateImage()
{
    // If not valid, copy just the local pixmap to the label
    if(!valid)
    {
        pixmapMutex.lock();
        ui->imageLabel->setPixmap(pixmap);
        pixmapMutex.unlock();
        return;
    }

    // Calculate the corners of the rectangle to draw
    QPoint uL((press.x() < release.x()) ? press.x() : release.x(), (press.y() < release.y()) ? press.y() : release.y());
    QPoint lR((press.x() > release.x()) ? press.x() : release.x(), (press.y() > release.y()) ? press.y() : release.y());
    QPoint lL(uL.x(), lR.y());
    QPoint uR(lR.x(), uL.y());

    // Generate the rectangle object
    QPolygon rect;
    rect.append(uL);
    rect.append(uR);
    rect.append(lR);
    rect.append(lL);
    rect.append(uL);

    // Create the pen to draw with
    QPen pen(Qt::red);
    pen.setWidth(3);

    // Lock the context and make a duplicate of the image
    pixmapMutex.lock();
    QPixmap copy(pixmap);
    pixmapMutex.unlock();

    // Draw the image over the new image copy
    QPainter painter(&copy);
    painter.setPen(pen);
    painter.drawPolyline(rect);

    // Update the imagelabel
    ui->imageLabel->setPixmap(copy);
}

void MainWindow::changeMode(int checkboxState)
{
    downloader->mode = (checkboxState == Qt::Checked) ? true : false;
}

void MainWindow::resetPtz()
{
    downloader->reset_ptz();
}

void MainWindow::changeTrackingMode(int checkboxState)
{
    downloader->set_tracking_enabled((checkboxState == Qt::Checked) ? 1 : 0);
}
// Hey Nathan, this is Chazz; I just wanted to say what's up!!
