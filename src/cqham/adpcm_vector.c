/* EA7IXM: prints the ADPCM block for a fixed test signal, to compare with
 * CQHAM's shared/adpcm.js byte by byte (CI only, not in the firmware). */
#include "cqham_adpcm.h"

#include <math.h>
#include <stdio.h>

int main(void) {
    int16_t pcm[160], back[160];
    uint8_t out[CQHAM_ADPCM_BYTES(160)];
    for (int i = 0; i < 160; i++) {
        pcm[i] = (int16_t)lrint(9000.0 * sin(i * 0.31) + 3000.0 * sin(i * 1.7));
    }
    size_t n = cqham_adpcm_encode(pcm, 160, out);
    for (size_t i = 0; i < n; i++) printf("%02x", out[i]);
    printf("\n");
    size_t m = cqham_adpcm_decode(out, n, back, 160);
    long   sum = 0;
    for (size_t i = 0; i < m; i++) sum += back[i];
    printf("decoded %zu sum %ld first %d last %d\n", m, sum, back[0], back[m - 1]);
    return 0;
}
