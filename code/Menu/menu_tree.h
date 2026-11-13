/* Static parent/child/circular-sibling menu tree, adapted from Qcar's old tree. */
#ifndef QCAR_LINKED_MENU_TREE_H
#define QCAR_LINKED_MENU_TREE_H
#include <stdint.h>
#ifndef MENU_TREE_CAPACITY
#define MENU_TREE_CAPACITY 160u
#endif
typedef void (*Menu_Draw)(void);
typedef void (*Menu_Keys)(uint8_t,uint8_t,uint8_t,uint8_t);
typedef enum {MENU_Folder,MENU_Parameter,MENU_Action,MENU_Back} Menu_Kind;
typedef struct Menu_Item {
 const char *name;const void *data;
 struct Menu_Item *Father,*First_Son,*Next_Brother,*Last_Brother,*cursor;
 Menu_Draw draw;Menu_Keys keys;
 uint16_t sons,rank,top;
 uint8_t page,group,kind,step_index;
} Menu_Item;
void Menu_Tree_Reset(void);
Menu_Item *Menu_Tree_Add(Menu_Item *parent,const char *name,Menu_Kind kind,uint8_t page,uint8_t group,const void *data);
Menu_Item *Menu_Tree_Child(Menu_Item *parent,uint16_t index);
void Menu_Tree_Move(Menu_Item *folder,int direction,uint16_t visible);
uint16_t Menu_Tree_Used(void);
float Menu_Edit_Step(const Menu_Item *node,float base,int integer,int fixed);
int Menu_Edit_Cycle(Menu_Item *node,float base,int integer,int fixed);
unsigned Menu_Edit_Decimals(float step,unsigned original);
#endif
