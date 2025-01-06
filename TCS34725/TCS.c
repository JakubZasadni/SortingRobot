uint8_t get_position_from_color(uint8_t r, uint8_t g, uint8_t b) {
    // Przykładowe progi kolorów dla 8 pozycji
    if (r > 200 && g < 100 && b < 100) return 0;  // Czerwony
    if (r > 200 && g > 200 && b < 100) return 1;  // Żółty
    if (r < 100 && g > 200 && b < 100) return 2;  // Zielony
    if (r < 100 && g > 200 && b > 200) return 3;  // Cyjan
    if (r < 100 && g < 100 && b > 200) return 4;  // Niebieski
    if (r > 200 && g < 100 && b > 200) return 5;  // Fioletowy
    if (r > 150 && g > 150 && b > 150) return 6;  // Biały
    return 7;  // Inne kolory lub brak koloru – pozycja 7
}
