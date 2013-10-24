// generated by Fast Light User Interface Designer (fluid) version 1.0011

#include "ScissorPanelUI.h"

inline void ScissorPanelUI::cb_Image_i(Fl_Round_Button*, void*)
{
    imgView->OrigImage();
}
void ScissorPanelUI::cb_Image(Fl_Round_Button* o, void* v)
{
    ((ScissorPanelUI*)(o->parent()->parent()->user_data()))->cb_Image_i(o, v);
}

inline void ScissorPanelUI::cb_contour_i(Fl_Round_Button*, void*)
{
    imgView->Contour();
}
void ScissorPanelUI::cb_contour(Fl_Round_Button* o, void* v)
{
    ((ScissorPanelUI*)(o->parent()->parent()->user_data()))->cb_contour_i(o, v);
}

inline void ScissorPanelUI::cb_Pixel_i(Fl_Round_Button*, void*)
{
    imgView->PixelColor();
}
void ScissorPanelUI::cb_Pixel(Fl_Round_Button* o, void* v)
{
    ((ScissorPanelUI*)(o->parent()->parent()->user_data()))->cb_Pixel_i(o, v);
}

inline void ScissorPanelUI::cb_Cost_i(Fl_Round_Button*, void*)
{
    imgView->CostGraph();
}
void ScissorPanelUI::cb_Cost(Fl_Round_Button* o, void* v)
{
    ((ScissorPanelUI*)(o->parent()->parent()->user_data()))->cb_Cost_i(o, v);
}

inline void ScissorPanelUI::cb_Path_i(Fl_Round_Button*, void*)
{
    imgView->PathTree();
}
void ScissorPanelUI::cb_Path(Fl_Round_Button* o, void* v)
{
    ((ScissorPanelUI*)(o->parent()->parent()->user_data()))->cb_Path_i(o, v);
}

inline void ScissorPanelUI::cb_Minimum_i(Fl_Round_Button*, void*)
{
    imgView->MinPath();
}
void ScissorPanelUI::cb_Minimum(Fl_Round_Button* o, void* v)
{
    ((ScissorPanelUI*)(o->parent()->parent()->user_data()))->cb_Minimum_i(o, v);
}

inline void ScissorPanelUI::cb_expanded_i(Fl_Counter*, void*)
{
    imgView->PartialExpanding();
}
void ScissorPanelUI::cb_expanded(Fl_Counter* o, void* v)
{
    ((ScissorPanelUI*)(o->parent()->parent()->user_data()))->cb_expanded_i(o, v);
}

inline void ScissorPanelUI::cb_whole_i(Fl_Round_Button*, void*)
{
    imgView->BrushSelection(0);
}
void ScissorPanelUI::cb_whole(Fl_Round_Button* o, void* v)
{
    ((ScissorPanelUI*)(o->parent()->parent()->user_data()))->cb_whole_i(o, v);
}

inline void ScissorPanelUI::cb_Brush_i(Fl_Round_Button*, void*)
{
    imgView->BrushSelection(1);
}
void ScissorPanelUI::cb_Brush(Fl_Round_Button* o, void* v)
{
    ((ScissorPanelUI*)(o->parent()->parent()->user_data()))->cb_Brush_i(o, v);
}

inline void ScissorPanelUI::cb_Close_i(Fl_Button*, void*)
{
    hide();
}
void ScissorPanelUI::cb_Close(Fl_Button* o, void* v)
{
    ((ScissorPanelUI*)(o->parent()->user_data()))->cb_Close_i(o, v);
}

ScissorPanelUI::ScissorPanelUI()
{
    Fl_Window* w;
    {
        Fl_Window* o = mainWindow = new Fl_Window(522, 191, "Scissor Panel UI");
        w = o;
        o->user_data((void*)(this));
        {
            Fl_Group* o = new Fl_Group(0, 0, 520, 156);
            o->align(FL_ALIGN_TOP_LEFT);
            {
                Fl_Box* o = new Fl_Box(1, 3, 95, 21, "Work Mode:");
                o->align(FL_ALIGN_LEFT | FL_ALIGN_INSIDE);
            }
            {
                Fl_Box* o = new Fl_Box(0, 24, 264, 32);
                o->box(FL_ENGRAVED_FRAME);
            }
            {
                Fl_Round_Button* o = new Fl_Round_Button(1, 26, 103, 28, "Image Only");
                o->type(102);
                o->down_box(FL_ROUND_DOWN_BOX);
                o->callback((Fl_Callback*)cb_Image);
            }
            {
                Fl_Round_Button* o = contour = new Fl_Round_Button(109, 26, 148, 28, "Image with Contour");
                o->type(102);
                o->down_box(FL_ROUND_DOWN_BOX);
                o->callback((Fl_Callback*)cb_contour);
            }
            {
                Fl_Box* o = new Fl_Box(1, 59, 95, 22, "Debug Mode:");
                o->align(FL_ALIGN_LEFT | FL_ALIGN_INSIDE);
            }
            {
                Fl_Box* o = new Fl_Box(0, 81, 520, 75);
                o->box(FL_ENGRAVED_FRAME);
            }
            {
                Fl_Round_Button* o = new Fl_Round_Button(1, 83, 107, 28, "Pixel Nodes");
                o->type(102);
                o->down_box(FL_ROUND_DOWN_BOX);
                o->callback((Fl_Callback*)cb_Pixel);
            }
            {
                Fl_Round_Button* o = new Fl_Round_Button(110, 83, 147, 28, "Cost Graph");
                o->type(102);
                o->down_box(FL_ROUND_DOWN_BOX);
                o->callback((Fl_Callback*)cb_Cost);
            }
            {
                Fl_Round_Button* o = new Fl_Round_Button(273, 83, 115, 28, "Path Tree");
                o->type(102);
                o->down_box(FL_ROUND_DOWN_BOX);
                o->callback((Fl_Callback*)cb_Path);
            }
            {
                Fl_Round_Button* o = new Fl_Round_Button(390, 83, 130, 28, "Minimum Path");
                o->type(102);
                o->down_box(FL_ROUND_DOWN_BOX);
                o->callback((Fl_Callback*)cb_Minimum);
            }
            {
                Fl_Counter* o = expanded = new Fl_Counter(112, 113, 304, 20, "Number of Expanded Nodes");
                o->minimum(0);
                o->maximum(0);
                o->step(1);
                o->callback((Fl_Callback*)cb_expanded);
                o->when(FL_WHEN_RELEASE);
            }
            o->end();
        }
        {
            Fl_Group* o = new Fl_Group(268, 0, 256, 56);
            o->align(FL_ALIGN_TOP_LEFT);
            {
                Fl_Box* o = new Fl_Box(268, 24, 252, 32);
                o->box(FL_ENGRAVED_FRAME);
            }
            {
                Fl_Round_Button* o = whole = new Fl_Round_Button(272, 26, 116, 28, "Whole Image");
                o->type(102);
                o->down_box(FL_ROUND_DOWN_BOX);
                o->selection_color(2);
                o->callback((Fl_Callback*)cb_whole);
            }
            {
                Fl_Round_Button* o = new Fl_Round_Button(389, 26, 127, 28, "Brush Selection");
                o->type(102);
                o->down_box(FL_ROUND_DOWN_BOX);
                o->selection_color(2);
                o->callback((Fl_Callback*)cb_Brush);
            }
            {
                Fl_Box* o = new Fl_Box(268, 2, 132, 22, "Scissor Range:");
                o->align(FL_ALIGN_LEFT | FL_ALIGN_INSIDE);
            }
            o->end();
        }
        {
            Fl_Button* o = new Fl_Button(189, 159, 152, 28, "Close");
            o->callback((Fl_Callback*)cb_Close);
        }
        o->end();
    }
}

void ScissorPanelUI::show()
{
    mainWindow->show();
}

void ScissorPanelUI::hide()
{
    mainWindow->hide();
}
