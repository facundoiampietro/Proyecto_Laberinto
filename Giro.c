// Direcciones hacia donde puede estar mirando el autito
#define NORTE 0
#define ESTE  1
#define SUR   2
#define OESTE 3

// Tipos de giro que puede hacer
#define ADELANTE 0
#define IZQUIERDA 1
#define DERECHA  2
#define GIRO_180   3

// Devuelve la dirección hacia donde hay que ir según la diferencia entre casillas
int8_t obtener_orientacion_N(int8_t casillaActual, int8_t casilla_N) {
    if (casilla_N == casillaActual + 1) return OESTE;
    if (casilla_N == casillaActual - 1) return ESTE;
    if (casilla_N == casillaActual + 4) return NORTE;
    if (casilla_N == casillaActual - 4) return SUR;

    return -1; // Movimiento no válido (no adyacente o fuera del tablero)
}

// Calcula el giro que debe hacer el autito para pasar de su orientación actual a la deseada
int8_t obtenerGiro(int8_t orientacionActual, int8_t orientacion_N) {
    int diferencia = (orientacion_N - orientacionActual + 4) % 4; //el %4 se queda con el resto de la divsion por 4

    switch (diferencia) {
        case 0: return ADELANTE;
        case 1: return DERECHA;
        case 2: return GIRO_U;
        case 3: return IZQUIERDA;
        default: return -1; // Error
    }
}
-----------------------------------------------------------------------------------------
// Aca iria la parte de la interrupcion que decide donde ir ya con los datos necesarios cargados


int orientacion_N = obtener_orientacion_N(casillaActual, casilla_N);
int giro = obtenerGiro(orientacionActual, orientacion_N);
orientacionActual = orientacion_N  ;  //actualizo la orientacion


