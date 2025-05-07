#include <esp_camera.h>
#include <WiFi.h>

const char* ssid = "your_SSID"; // Replace with your WiFi SSID
const char* password = "your_PASSWORD"; // Replace with your WiFi password

#define CAMERA_MODEL_AI_THINKER // Define the camera model

#include "camera_pins.h" // Include the camera pins header file

// Start server
WifiServer server(80); // Create a server object on port 80

void setup() {
    Serial.begin(115200); // Initialize serial communication for debugging  
    delay(1000); // Wait for the serial connection to establish

    // Connect to WiFi
    WiFi.begin(ssid, password); // Start the WiFi connection
    Serial.print("Connecting to WiFi");

    while (WiFi.status() != WL_CONNECTED) { // Wait for the connection to establish
        delay(1000); // Wait for 1 second before checking again
        Serial.print("."); // Print a dot to indicate progress
    }

    Serial.println("Connected to WiFi"); // Print a message when connected
    Serial.print("IP Address: "); // Print the IP address of the ESP32
    Serial.println(WiFi.localIP()); // Print the local IP address

    // Start the camera
    if (!camera.begin()) {
        Serial.println("Camera initialization failed"); // Print an error message if camera initialization fails
        return; // Exit the setup function
    }

    // Start HTTP server
    server.begin(); // Start the server
    Serial.println("Server started"); // Print a message when the server starts
}

void loop() {
    WiFiClient client = server.available(); // Check if a client is connected

    if (!client) {
        return; // If no client is connected, return
    }

    // Wait fo the client to send data
    Serial.println("New client."); // Print a message when a new client connects  
    String currentLine = "";

    while (client.connected()) {
        if (client.available()) {
            char c = client.read(); 
            Serial.write(c); // Print the received character to the serial monitor
            currentLine += c; // Append the character to the current line

            if (c == '\n') {
                if (currentLine.length() == 0) {
                    client.println("HTTP/1.1 200 OK"); // Send HTTP response header
                    client.println("Content-Type: multipart/x-mixed-replace; boundary=frame"); // Set content type to HTML
                    client.println(); // Send a blank line to indicate the end of the header

                    // Start streaming video frames
                    while (true) {
                        // Capture streaming 
                        camera_fb_t * fb = NULL;
                        fb = camera.capture(); // Capture a frame from the camera

                        if (!fb) {
                            Serial.println("Camera capture failed"); // Print an error message if capture fails
                            break; // Exit the loop if capture fails
                        }

                        // Send HTTP response header for streaming
                        client.println("--frame"); // Send boundary to indicate the start of a new frame
                        client.println("Content-Type: image/jpeg"); // Set content type to JPEG
                        client.println("Content-Length: " + String(fb->len)); // Send the length of the image data
                        client.println(); // Send a blank line to indicate the end of the header
                        client.write(fb->buf, fb->len); // Send the image data to the client
                        client.println(); // Send a blank line to indicate the end of the image data
                        camera.returnFrameBuffer(fb); // Return the frame buffer to the camera
                        delay(100); // Wait for 100 milliseconds before capturing the next frame
                    }
                }
                break; // Exit the loop if a new line is received
            }
            else if (currentLine.endsWith("GET /capture")) {
                // Capture a single frame
                camera_fb_t * fb = camera.capture(); // Capture a frame from the camera

                if (!fb) {
                    Serial.println("Camera capture failed"); // Print an error message if capture fails
                    break; // Exit the loop if capture fails
                }

                client.println("HTTP/1.1 200 OK"); // Send HTTP response header
                client.println("Content-Type: image/jpeg"); // Set content type to JPEG
                client.println("Content-Length: " + String(fb->len)); // Send the length of the image data
                client.println(); // Send a blank line to indicate the end of the header
                client.write(fb->buf, fb->len); // Send the image data to the client
                camera.returnFrameBuffer(fb); // Return the frame buffer to the camera
            }
        }
    }

    client.stop(); // Stop the client connection
    Serial.println("Client disconnected"); // Print a message when the client disconnects
}