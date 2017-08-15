//
// Created by Stanislav Olekhnovich on 02/08/2017.
//

#include "WebSocketMessageHandler.h"

string WebSocketMessageHandler::CreateResponseMessage(ControllerOutput controllerOutput)
{
    json msgJson;

    msgJson["steering_angle"] = controllerOutput.SteeringAngle;
    msgJson["throttle"] = controllerOutput.Throttle;

    // Green
    msgJson["mpc_x"] = controllerOutput.PredictedPathX;
    msgJson["mpc_y"] = controllerOutput.PredictedPathY;

    // Yellow
    msgJson["next_x"] = controllerOutput.ReferencePathX;
    msgJson["next_y"] = controllerOutput.ReferencePathY;

    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
    return msg;
}

string WebSocketMessageHandler::ProcessMessageContent(string& content)
{
    auto jsonContent = json::parse(content);
    string eventType = jsonContent[0].get<string>();

    string response;

    if (eventType == "telemetry")
    {
        auto eventData = jsonContent[1];

        auto controlData = ReadControlMeasurementFromJson(eventData);

        ControllerOutput output = controller.Process(controlData);

        response = CreateResponseMessage(output);
    }
    return response;
}


ControlData WebSocketMessageHandler::ReadControlMeasurementFromJson(json data)
{
    std::vector<double> refPathX = data["ptsx"];
    std::vector<double> refPathY = data["ptsy"];

    double px = data["x"];
    double py = data["y"];
    double psi = data["psi"];
    double v = data["speed"];
    double a = data["throttle"];
    double angle = data["steering_angle"];

    return {refPathX, refPathY, psi, px, py, v, a, angle};
}


string WebSocketMessageHandler::GetMessageContent(const string& message)
{
    string content;

    bool hasNullContent = (message.find("null") != string::npos);
    if (hasNullContent)
        return content;

    auto b1 = message.find_first_of('[');
    auto b2 = message.find_last_of(']');

    if (b1 != string::npos && b2 != string::npos)
        content = message.substr(b1, b2 - b1 + 1);

    return content;
}

bool WebSocketMessageHandler::MessageHasExpectedPrefix(const string& message)
{
    // "42" at the start of the message means there's a websocket message event.
    const string prefix {"42"};
    return (message.substr(0, prefix.size()) == prefix);
}

void WebSocketMessageHandler::HandleMessage(const string& message, uWS::WebSocket<uWS::SERVER>& ws)
{
    if (!MessageHasExpectedPrefix(message))
        return;

    auto content = GetMessageContent(message);
    if (content.empty())
    {
        SendDefaultResponse(ws);
        return;
    }

    auto response = ProcessMessageContent(content);

//    std::cout << content << std::endl;
//    std::cout << response << std::endl;

    if (!response.empty())
        ws.send(response.data(), response.length(), uWS::OpCode::TEXT);
}

void WebSocketMessageHandler::SendDefaultResponse(uWS::WebSocket<uWS::SERVER>& ws) const
{
    string response = "42[\"manual\",{}]";
    std::cout << response << std::endl;
    ws.send(response.data(), response.length(), uWS::TEXT);
}


