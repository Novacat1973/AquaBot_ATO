#include "Org_01.h"
#include "TomThumb.h"

void updateDisplay() {
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(2);
    display.setFont(&Org_01);
    display.setCursor(3, 11);
    display.println("Set Volume");
    display.drawLine(2, 15, 126, 15, 1);
    display.setTextSize(3);
    display.setCursor(0, 42);
    display.setTextWrap(0);
    display.setCursor(47, 42);
    display.println("48");
    display.setTextSize(2);
    display.setFont(&TomThumb);
    display.setCursor(88, 44);
    display.print("49");
    display.setCursor(25, 44);
    display.print("47");
    display.setTextSize(1);
    display.setCursor(10, 44);
    display.print("46");
    display.setCursor(111, 44);
    display.print("50");
    display.fillTriangle(59, 58, 69, 58, 64, 53, 1);
    display.display();
}
