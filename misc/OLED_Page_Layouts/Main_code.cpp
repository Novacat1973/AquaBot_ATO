#include "Org_01.h"
#include "TomThumb.h"

void updateDisplay() {
    display.clearDisplay();
    display.setFont(&Org_01);
    display.setTextSize(2);
    display.setCursor(5, 17);
    display.println("Filling up");
    display.setTextSize(1);
    display.setCursor(5, 34);
    display.println("Evaporation: 10 L/w");
    display.setCursor(5, 45);
    display.println("Remaining: 10/20 L");
    display.setCursor(5, 56);
    display.println("Empty in: 3 d (LW)");
    display.setFont(&TomThumb);
    display.setCursor(107, 10);
    display.println("100 %");
    display.drawRect(107, 13, 16, 47, 1);
    display.fillRect(109, 35, 12, 23, 1);
    display.display();
}
