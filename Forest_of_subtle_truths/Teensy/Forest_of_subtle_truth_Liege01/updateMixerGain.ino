void updateMixerGain() {
  
  if (millis() > lasttime_fade_update + fade_update_interval) {
    AudioNoInterrupts();
    ////////////////////////// update mixer_gain
    for (int i = 0; i < 4; i++) {
      if (mixer_gain[i] > destination_gain[i]) mixer_gain[i] = mixer_gain[i] - 0.02;
      if (mixer_gain[i] < destination_gain[i]) mixer_gain[i] = mixer_gain[i] + 0.02;
      if (mixer_gain[i] > 1) mixer_gain[i] = 1;
      if (mixer_gain[i] < 0.01) mixer_gain[i] = 0;
    }
    
    ///////////////////////// set mixer_gain
    // mixerMono
    for (int i = 0; i < 4; i++) {
      mixerL.gain(i, mixer_gain[i]); // L
      mixerR.gain(i, mixer_gain[i]); // R
     // Serial.print(mixer_gain[i]); Serial.print(", ");
      
   }
  //  Serial.println();
    lasttime_fade_update = millis();
    AudioInterrupts();
  }

}
