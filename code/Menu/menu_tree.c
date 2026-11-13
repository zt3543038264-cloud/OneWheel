#include "menu_tree.h"
#include <string.h>
#include <math.h>
#if defined(__TASKING__)
#pragma section all "cpu0_dsram"
#endif
static Menu_Item pool[MENU_TREE_CAPACITY];static uint16_t used;
void Menu_Tree_Reset(void){memset(pool,0,sizeof(pool));used=0;}
uint16_t Menu_Tree_Used(void){return used;}
Menu_Item *Menu_Tree_Add(Menu_Item *p,const char *name,Menu_Kind kind,uint8_t page,uint8_t group,const void *data)
{
 Menu_Item *n,*last;
 if(used>=MENU_TREE_CAPACITY||(p&&p->kind!=MENU_Folder))return 0;
 n=&pool[used++];n->step_index=2;n->name=name;n->kind=(uint8_t)kind;n->page=page;n->group=group;n->data=data;n->Father=p;
 if(p){
  n->rank=p->sons++;
  if(!p->First_Son){p->First_Son=p->cursor=n;n->Next_Brother=n->Last_Brother=n;}
  else{last=p->First_Son->Last_Brother;n->Next_Brother=p->First_Son;n->Last_Brother=last;last->Next_Brother=n;p->First_Son->Last_Brother=n;}
 }
 return n;
}
Menu_Item *Menu_Tree_Child(Menu_Item *p,uint16_t index)
{
 Menu_Item *n;if(!p||index>=p->sons)return 0;
 n=p->First_Son;while(index--)n=n->Next_Brother;return n;
}
void Menu_Tree_Move(Menu_Item *p,int direction,uint16_t visible)
{
 if(!p||!p->cursor||!visible)return;
 p->cursor=direction>0?p->cursor->Next_Brother:p->cursor->Last_Brother;
 if(p->cursor->rank<p->top)p->top=p->cursor->rank;
 if(p->cursor->rank>=p->top+visible)p->top=p->cursor->rank-visible+1;
}
#if defined(__TASKING__)
#pragma section all restore
#endif

static const float edit_scales[5]={.01f,.1f,1.0f,10.0f,100.0f};
float Menu_Edit_Step(const Menu_Item *n,float base,int integer,int fixed)
{
 float step;if(!isfinite(base)||base<=0)return 0;
 if(fixed)return base;
 step=base*edit_scales[n&&n->step_index<5?n->step_index:2];
 if(integer){step=roundf(step);if(step<1)step=1;}
 return step;
}
int Menu_Edit_Cycle(Menu_Item *n,float base,int integer,int fixed)
{
 unsigned i,index;float current;
 if(!n||fixed||!isfinite(base)||base<=0)return 0;
 current=Menu_Edit_Step(n,base,integer,0);index=n->step_index<5?n->step_index:2;
 for(i=0;i<5;++i){
  index=(index+1u)%5u;
  if(integer&&base*edit_scales[index]<1)continue;
  if(base*edit_scales[index]!=current){n->step_index=(uint8_t)index;return 1;}
 }
 return 0;
}
unsigned Menu_Edit_Decimals(float step,unsigned original)
{
 unsigned n=0;if(!isfinite(step)||step<=0)return original;
 while(n<8&&(step<.99999f||fabsf(step-roundf(step))>.0001f)){step*=10;n++;}
 return n>original?n:original;
}
