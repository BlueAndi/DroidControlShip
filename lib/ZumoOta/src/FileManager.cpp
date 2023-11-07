/* MIT License
 *
 * Copyright (c) 2023 Andreas Merkle <web@blue-andi.de>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*******************************************************************************
    DESCRIPTION
*******************************************************************************/
/**
 * @brief  FileManager realization
 * 
 */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "FileManager.h"

/******************************************************************************
 * Compiler Switches
 *****************************************************************************/

/******************************************************************************
 * Macros
 *****************************************************************************/

/******************************************************************************
 * Types and classes
 *****************************************************************************/

/******************************************************************************
 * Prototypes
 *****************************************************************************/

/******************************************************************************
 * Local Variables
 *****************************************************************************/

/******************************************************************************
 * Public Methods
 *****************************************************************************/

FileManager::FileManager() {
    
}
    

FileManager::~FileManager() {
}
   

bool FileManager::createUploadPage() {
    if(!LittleFS.begin()){
        Serial.println("Fehler beim Starten von LittleFS");
        return false;
    }

    File file = LittleFS.open("/test.html", "w");
    if(!file){
        Serial.println("Fehler beim Erstellen der Datei");
        return false;
    }

    String content = R"rawliteral(
    <!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Upload</title>
    <style>
        body {
            display: flex;
            flex-direction: column;
            justify-content: center;
            align-items: center;
            min-height: 100vh;
            margin: 0;
            font-family: Arial, Helvetica, sans-serif;
            
            background-image: url('https://www.robot-maker.com/shop/1252-thickbox_default/robot-pololu-zumo-32u4.jpg');
            background-position: center;
            background-repeat: no-repeat;
            background-size: cover;
            background-color: rosybrown;
        }

        .upload-form {
            display: flex;
            flex-direction: column;
            align-items: center;
            padding: 60px;
            border: 8px solid black;
            border-radius: 10px;
            background-color: white;
        }

        .upload-form input[type="file"] {
            margin-bottom: 10px;
            padding: 10px;
            border-radius: 5px;
            border: 1px solid #ccc;
        }

        .upload-form input[type="submit"] {
            padding: 10px 20px;
            border-radius: 20px;
            border: none;
            background-color: lightblue;
            cursor: pointer;
        }
        h1, p {
            margin-top: 0;
            color: yellow; 
        }
        
    </style>
</head>
<body>
    <h1>Willkommen zur Upload Seite!</h1>
    <p>Bitte Anweisungen folgen!</p>
    <p> Bitte ein Dokument einreichen und auf Upload drücken!</p>
    <form class="upload-form" method="post" action="/upload" enctype="multipart/form-data">
        <input type="file" name="firmware">
        <input type="submit" value="Upload">
    </form>
    
</body>
</html>
)rawliteral";

    if (file.print(content)) {
        Serial.println("Datei erfolgreich erstellt");
    } else {
        Serial.println("Fehler beim Schreiben der Datei");
        return false;
    }

    file.close();
    return true;
}

String FileManager:: readUploadPage(){
    if(!LittleFS.begin()){
        Serial.println("Fehler beim Starten von LittleFS");
        return "";
    }

    File uploadPage = LittleFS.open("/test.html", "r");
    if (!uploadPage) {
        Serial.println("Fehler beim Öffnen der Upload-Seite.");
        return "";
    }
    
    String pageContent = uploadPage.readString();
    uploadPage.close();

    return pageContent;

    /*if (!LittleFS.begin()) {
        Serial.println("Fehler beim Starten von LittleFS");
        return "";
    }

    const uint32_t maxBufferSize = 1024; // Max. Anzahl von Bytes im Puffer
    char buffer[maxBufferSize]; // Puffer, um die Datei zu lesen
    FileReader fileReader;

    size_t bytesRead = fileReader.readFile("/test.html", buffer, maxBufferSize);
    if (bytesRead == 0) {
        Serial.println("Fehler beim Lesen der Upload-Seite.");
        return "";
    }

    // Konvertiere den Puffer in einen String
    String pageContent = String(buffer);

    return pageContent;*/

}

    





