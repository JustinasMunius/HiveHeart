#include <Arduino.h>
#include <HTTPClient.h>
#include "Application.h"
#include "state_machine/DetectWakeWordState.h"
#include <ArduinoJson.h>
#include "config.h"

Application::Application(I2SSampler *sample_provider)
{
    // detect wake word state - waits for the wake word to be detected
    m_detect_wake_word_state = new DetectWakeWordState(sample_provider);
    // start off in the detecting wakeword state
    m_current_state = m_detect_wake_word_state;
    m_current_state->enterState();
}

// process the next batch of samples
void Application::run()
{
    HTTPClient client;

    bool state_done = m_current_state->run();
    if (state_done)
    {
        m_current_state->exitState();

        DynamicJsonDocument doc(1024);
        doc["content"] = "Avilyje isgirstas nustatytas garsas";
        doc["level"] = 2;
        doc["hive"] = HIVE_ID;

        String pbody;
        serializeJson(doc, pbody);

        client.begin("https://hivemanager.azurewebsites.net/api/notifications");
        client.addHeader("Content-Type", "application/json");
        int httpCode = client.POST(pbody);

        if (httpCode > 0)
        {
          String response = client.getString();
          Serial.println("\nStatuscode: " + String(httpCode));
          Serial.println(response);
        }
        else {
          Serial.println("Error on HTTP request");
        }

        m_current_state->enterState();
    }
    vTaskDelay(10);
}
