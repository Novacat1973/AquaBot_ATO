#include "Org_01.h"

void updateDisplay() {
    display.clearDisplay();
    display.setTextSize(2);
    display.setFont(&Org_01);
    display.setCursor(0, 28);
    display.setTextWrap(0);
    display.setCursor(1, 28);
    display.println("Reservoir Full");
    display.drawLine(1, 33, 127, 33, 1);
    display.setTextSize(1);
    display.setFont(NULL);
    display.setCursor(0, 40);
    display.setTextWrap(0);
    display.setCursor(43, 40);
    display.println("AWESOME");
    display.display();
}
