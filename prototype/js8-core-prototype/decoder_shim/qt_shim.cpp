#include <QLoggingCategory>

#include "JS8_Include/commons.h"

#include <mutex>

QLoggingCategory &decoder_js8() {
    static QLoggingCategory category;
    return category;
}

struct dec_data dec_data {};
struct specData specData {};
std::mutex fftw_mutex;
