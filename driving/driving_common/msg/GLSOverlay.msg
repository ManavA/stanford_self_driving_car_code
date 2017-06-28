Header header
uint8 coordinates
float64 origin_x
float64 origin_y
float64 origin_z
uint8[] cmd_buf
bool first_vertex
string name
float64 timestamp

uint8 GLOBAL_COORDINATES = 1
uint8 SMOOTH_COORDINATES = 2
uint8 LOCAL_COORDINATES  = 3

uint8 POINTS             = 0
uint8 LINES              = 1
uint8 LINE_STRIP         = 2
uint8 LINE_LOOP          = 3
uint8 TRIANGLES          = 4
uint8 TRIANGLE_STRIP     = 5
uint8 TRIANGLE_FAN       = 6
uint8 QUADS              = 7
uint8 QUAD_STRIP         = 8
uint8 POLYGON            = 9

uint8 END                = 10
uint8 VERTEX3            = 11
uint8 COLOR              = 12
uint8 PUSH_MATRIX        = 13
uint8 POP_MATRIX         = 14
uint8 ROTATEF            = 15
uint8 TRANSLATEF         = 16
uint8 SCALEF             = 17
uint8 LINEWIDTH          = 18
uint8 POINTSIZE          = 19
uint8 RENDERSTROKESTRING = 20
uint8 DRAW_ARROW         = 21
uint8 VERTEX2            = 22
uint8 COLOR4             = 23
uint8 CIRCLE             = 24
uint8 SQUARE             = 25
uint8 CARPET             = 26
uint8 COLOR_CARPET       = 27

uint8 LINE_STIPPLE       = 28
uint8 ENABLE             = 29
uint8 DISABLE            = 30

uint8 DEPTH_MASK         = 31

