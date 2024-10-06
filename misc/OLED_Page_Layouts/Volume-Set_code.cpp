#include "Org_01.h"

void updateDisplay() {
    display.clearDisplay();
    display.setTextSize(2);
    display.setFont(&Org_01);
    display.setCursor(0, 28);
    display.setTextWrap(0);
    display.setCursor(13, 28);
    display.println("Volume Set");
    display.drawLine(11, 33, 116, 33, 1);
    display.setTextSize(1);
    display.setFont(NULL);
    display.setCursor(0, 40);
    display.setTextWrap(0);
    display.setCursor(49, 40);
    display.println("GREAT");
    display.display();
}
