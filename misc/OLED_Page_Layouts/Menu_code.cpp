#include "Org_01.h"

void updateDisplay() {
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(2);
    display.setFont(&Org_01);
    display.setCursor(3, 11);
    display.println("Menu");
    display.drawLine(2, 15, 126, 15, 1);
    display.fillTriangle(5, 25, 5, 35, 10, 30, 1);
    display.setFont(NULL);
    display.setTextSize(1);
    display.setCursor(20, 27);
    display.println("Reservoir Full");
    display.fillTriangle(5, 45, 5, 55, 10, 50, 1);
    display.setCursor(20, 47);
    display.println("Set Volume");
    display.display();
}
