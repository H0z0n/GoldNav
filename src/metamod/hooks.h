// POST-хуки DLL_FUNCTIONS, зарегистрированные в meta_api.cpp::GetEntityAPI2_Post.
// Сигнатуры должны точно соответствовать DLL_FUNCTIONS из HLSDK eiface.h.
#pragma once

// edict_t — typedef, не отдельный struct-тег; forward declaration конфликтует с
// реальным `typedef struct edict_s edict_t;` из HLSDK, поэтому подключаем заголовок.
#include <extdll.h>

namespace goldnav {

void Hook_ServerActivate_Post(edict_t* pEdictList, int edictCount, int clientMax);
void Hook_ServerDeactivate_Post();
void Hook_StartFrame_Post();

} // namespace goldnav
