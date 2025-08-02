#pragma once
#include <string>

void startRecording(const char* filename);
void stopRecording();
void playbackRecording(const std::string& filename);