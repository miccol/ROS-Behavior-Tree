#include<Draw.h>

#include <X11/Xlib.h>

BT::ControlNode* tree;
bool init = false;


void * font_array[3] = {GLUT_BITMAP_8_BY_13,GLUT_BITMAP_8_BY_13,GLUT_BITMAP_8_BY_13};
void * font = font_array[0];

float x = 0.0;
float y = 0.4;
float x_offset = 0.01;
float r_color = 1;
float g_color = 1;
float b_color = 1;
GLfloat x_space = 0.06;

int depth;

double zoom = 1.0f;

float fraction = 0.1f;
float zoom_fraction =0.1f;



void drawString (void * font, char *s, float x, float y, float z)
{
     unsigned int i;
     glRasterPos3f(x, y, z);

     for (i = 0; i < 2; i++)
          glutBitmapCharacter (font, s[i]);
}


// draw the node itself using a solid square (color coded)



void renderBitmapString(float x, float y, void *font,const char *string)
{
    const char *c;
    glRasterPos2f(x, y);
    for (c=string; *c != '\0'; c++) {
        glutBitmapCharacter(font, *c);
    }
}



void draw_node(float x, float y, int node_type, const char *leafName, int status)
{

    float NODE_WIDTH = 0.021;
    float NODE_HEIGHT = 0.02;
    switch (node_type)
    {
    case BT::SELECTORSTAR:
        drawString(font, "?*", (x + NODE_WIDTH - 0.035), (y - NODE_HEIGHT/2), 0);
        break;
    case BT::SEQUENCESTAR:
        drawString(font, ">*", (x - NODE_WIDTH + 0.01 ), (y - NODE_HEIGHT/2), 0);
        break;

    case BT::SELECTOR:
        drawString(font, "?", (x + NODE_WIDTH - 0.025), (y - NODE_HEIGHT/2), 0);
        break;
    case BT::SEQUENCE:
        drawString(font, ">", (x - NODE_WIDTH + 0.015), (y - NODE_HEIGHT/2), 0);
        break;
    case BT::PARALLEL:
        drawString(font, "=", (x - NODE_WIDTH + 0.01), (y - NODE_HEIGHT/2), 0);
        break;
    case BT::DECORATOR:
        drawString(font, "D", (x - NODE_WIDTH + 0.01), (y - NODE_HEIGHT/2), 0);
        break;
    case BT::ACTION:
       {
            std::string st(leafName,0, 15);
            NODE_WIDTH = 0.01;
            for (unsigned int i = 0; i < st.size(); i++)
              NODE_WIDTH +=  0.01;
        }
        renderBitmapString((x - NODE_WIDTH +0.015), (y - NODE_HEIGHT/2), font,leafName);
        glColor3f(0.2, 1.0, 0.2);
        break;
    case BT::CONDITION:
    {
         std::string st(leafName,0, 15);

         NODE_WIDTH = 0.01;
         for (unsigned int i = 0; i < st.size(); i++)
           NODE_WIDTH +=  0.01;


     }
        renderBitmapString((x - NODE_WIDTH +0.015), (y - NODE_HEIGHT/2), font,leafName);
        break;
    default: break;
    }

    switch (status)
    {
        case RUNNING:   glColor3f(0.8, 0.8, 0.8); break;
        case SUCCESS:   glColor3f(0.0, 1.0, 0.0); break;
        case FAILURE:   glColor3f(1.0, 0.0, 0.0); break;
        case IDLE:      glColor3f(0.0, 0.0, 0.0); break;
        default: break;
    }


    glBegin(GL_LINE_LOOP);
    glVertex3f((GLfloat) (x + NODE_WIDTH), (GLfloat) (y - NODE_HEIGHT), (GLfloat) 0.0);
    glVertex3f((GLfloat) (x + NODE_WIDTH), (GLfloat) (y + NODE_HEIGHT), (GLfloat) 0.0);
    glVertex3f((GLfloat) (x - NODE_WIDTH), (GLfloat) (y + NODE_HEIGHT), (GLfloat) 0.0);
    glVertex3f((GLfloat) (x - NODE_WIDTH), (GLfloat) (y - NODE_HEIGHT), (GLfloat) 0.0);
    glColor3f(0.0, 0.0, 0.0);
    glEnd();
}

// draw the edge connecting one node to the other
void draw_edge(GLfloat parent_x, GLfloat parent_y, GLfloat parent_size, GLfloat child_x, GLfloat child_y, GLfloat child_size)
{
    glLineWidth(1.5);
    glColor3f(0.0, 0.0, 0.0);
    glBegin(GL_LINES);
    glVertex3f(parent_x, parent_y-parent_size, 0.0);
    glVertex3f(child_x, child_y+child_size, 0);
    glEnd();
}


// Keyboard callback function ( called on keyboard event handling )
void keyboard(unsigned char key, int x, int y)
{
    if (key == 'q' || key == 'Q')
        //youbot_common::stopSimulation();
        exit(EXIT_SUCCESS);
}


void resize(int width, int height) {
        // we ignore the params and do:

//    glutGet(GLUT_WINDOW_WIDTH);
//    glutGet(GLUT_WINDOW_HEIGHT);
//    glutReshapeWindow(glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT));




               //The far z clipping coordinate

}





void drawCircle(float radius)
{
   glBegin(GL_LINE_LOOP);

   for (int i=0; i<= 360; i++)
   {

      float degInRad = i*3.14142/180;
      glVertex2f(cos(degInRad)*radius,sin(degInRad)*radius);
   }

   glEnd();
}




float get_x_pos_at(BT::TreeNode* tree, int depth)
{
    if (depth == 0)
    {
    return tree->GetXPose();
    }
    else
    {
        BT::ControlNode* d = dynamic_cast<BT::ControlNode*> (tree);
        if (d == NULL)
        {
            return tree->GetXPose();

        }
        else
        {
            std::vector<BT::TreeNode*> children = d->GetChildren();

                return get_x_pos_at(children.back(), depth - 1);

        }
    }


}


void setpositions(BT::TreeNode* tree, GLfloat x_pos, GLfloat y_pos, GLfloat x_offset, GLfloat y_offset )
{

    //x_offset*pow(2,tree->GetDepth()-1)

    BT::ControlNode* d = dynamic_cast<BT::ControlNode*> (tree);
    if (d == NULL)
    {//if it is a leaf node, draw it

        float NODE_WIDTH = 0.03;
        std::string st(tree->get_name(),0, 15);
        NODE_WIDTH = 0.01;
        for (unsigned int i = 0; i < st.size(); i++)
          NODE_WIDTH +=  0.01;

     //   std::cout << "setting position of " << tree->name << " as " << x_pos + 2*NODE_WIDTH + x_space << std::endl;

        tree->SetXPose(x_pos + 2*NODE_WIDTH + x_space);
    }
    else
    {//if it is a control flow node, draw it and its children
        std::vector<BT::TreeNode*> children = d->GetChildren();
        int M = d->GetChildrenNumber();
        GLfloat x_min = 0.0;
        GLfloat x_max = 0.0;
        GLfloat x_shift = x_pos;
        GLfloat x_shift_new = 0.0;

        for (int i = 0; i < M; i++)
        {

            if (i > 0)
            {
                x_shift = get_x_pos_at(children[i - 1], children[i]->GetDepth());

            }
            setpositions(children[i], x_shift, y_pos - y_offset , x_offset  ,y_offset );


            x_shift = x_shift_new;
            // draw_edge((x_min+x_max)/2, y_pos, 0.02, x_pos - x_offset * (M-1) + 2*x_offset*(i) , y_pos - y_offset, 0.02);
        }


        if (children.size() % 2)
        {//odd number of children , the node goes on top the middle node

            int middle_child = ceil(children.size()/2);
            tree->SetXPose(children[middle_child]->GetXPose());

        }
        else
        {
            x_min = children.front()->GetXPose();
            x_max = children.back()->GetXPose();
            tree->SetXPose((x_min+x_max)/2);

        }
        //draw_node((GLfloat) (x_min+x_max)/2, (GLfloat) y_pos, tree->GetType(), tree->name.c_str(), tree->ReadColorState());

        //return x_shift_new + (x_min+x_max)/2;

    }

   // std::cout << "position of " << tree->name << " is " << tree->GetXPose() << std::endl;
}






void updateTree(BT::TreeNode* tree, GLfloat x_pos, GLfloat y_pos, GLfloat y_offset )
{

    //x_offset*pow(2,tree->GetDepth()-1)
   // GLfloat x_space = 0.01;

    BT::ControlNode* d = dynamic_cast<BT::ControlNode*> (tree);
    if (d == NULL)
    {//if it is a leaf node, draw it


        draw_node((GLfloat) tree->GetXPose() , (GLfloat) y_pos, tree->GetType(), tree->get_name().c_str(), tree->ReadColorState());

    }
    else
    {//if it is a control flow node, draw it and its children
        std::vector<BT::TreeNode*> children = d->GetChildren();
        int M = d->GetChildrenNumber();
        GLfloat x_min = 0.0;
        GLfloat x_max = 0.0;
      //  GLfloat x_space = 0.05;
        GLfloat x_shift = x_pos;
        GLfloat x_shift_new = 0.0;

        for (int i = 0; i < M; i++)
        {
             updateTree(children[i], x_shift, y_pos - y_offset  ,y_offset );
             draw_edge(tree->GetXPose(), y_pos, 0.02, children[i]->GetXPose() , y_pos - y_offset, 0.02);


        }


        draw_node((GLfloat) tree->GetXPose(), (GLfloat) y_pos, tree->GetType(), tree->get_name().c_str(), tree->ReadColorState());

        //return x_shift_new + (x_min+x_max)/2;

    }
}




void display()
{
    boost::this_thread::sleep(boost::posix_time::milliseconds(5)); //interrupt point


    glClearColor( r_color, g_color, b_color, 0.1);

    // clear the draw buffer .
    glClear(GL_COLOR_BUFFER_BIT);   // Erase everything
    setpositions(tree, x , y, x_offset , 0.1 );
    //exit(5);
    updateTree(tree, x , y , 0.1 );
   // std::cout << "TREE updated" << std::endl;
    glutSwapBuffers();
    glutPostRedisplay();

}


void processSpecialKeys(int key, int xx, int yy) {



    switch (key) {
        case GLUT_KEY_UP :
            y +=  fraction;
            break;
        case GLUT_KEY_DOWN :
            y -=  fraction;
            break;
        case GLUT_KEY_LEFT:
            x -=  fraction;
            break;
        case GLUT_KEY_RIGHT:
            x +=  fraction;
            break;
        case  GLUT_KEY_PAGE_UP:
         x_space +=  fraction;
            break;
        case  GLUT_KEY_PAGE_DOWN:
            x_space -=  fraction;
            break;
        case  GLUT_KEY_F1:
        if (r_color < 1)  r_color +=  fraction;
             break;
        case  GLUT_KEY_F2:
        if (r_color > 0) r_color -=  fraction;
            break;
        case  GLUT_KEY_F3:
        if (g_color < 1) g_color +=  fraction;
             break;
        case  GLUT_KEY_F4:
        if (g_color > 0) g_color -=  fraction;
            break;
        case  GLUT_KEY_F5:
        if (b_color < 1) b_color +=  fraction;
             break;
        case  GLUT_KEY_F6:
        if (b_color > 0) b_color -=  fraction;
            break;
        case GLUT_KEY_HOME:
        if (zoom < 1.0f)
        {
            glScalef( 1.0f  +zoom_fraction ,1.0f  +zoom_fraction,1.0f );
            zoom +=zoom_fraction;
        }else
        {
            glScalef( 1.0f,1.0f,1.0f );

        }
            break;
        case GLUT_KEY_END:
        glScalef( 1.0f  - zoom_fraction,1.0f  - zoom_fraction,1.0f );
        zoom -=zoom_fraction;

        break;


    }
}






void mouse(int button, int state, int x, int y)
{
   // Wheel reports as button 3(scroll up) and button 4(scroll down)
   if ((button == 1) || (button == 2)) // It's a wheel event
   {
       // Each wheel event reports like a button click, GLUT_DOWN then GLUT_UP
       if (state == GLUT_UP) return; // Disregard redundant GLUT_UP events
     //  printf("Scroll %s At %d %d\n", (button == 3) ? "Up" : "Down", x, y);
       exit(9);
   }else{  // normal button event
      // printf("Button %s At %d %d\n", (state == GLUT_DOWN) ? "Down" : "Up", x, y);
   }
}


void drawTree(BT::ControlNode* tree_)
{
    //***************************BT VISUALIZATION****************************
    int argc = 1;
    char *argv[1] = {(char*)"Something"};

    if (!init)
    {
        XInitThreads();
        glutInit(&argc, argv);      // Initialize GLUT
        init = true;
    }
    tree = tree_;
    depth = tree->GetDepth();

    glutInitWindowSize(1024,860);

    glutCreateWindow("Behavior Tree");  // Create a window
    //glutMouseFunc(mouse);

    glutReshapeFunc(resize);



    glClearColor( 0, 0.71, 0.00, 0.1);
    glutDisplayFunc(display);   // Register display callback


    glutKeyboardFunc(keyboard); // Register keyboard callback
    glutSpecialFunc(processSpecialKeys); //Register keyboard arrow callback

    glutMainLoop();             // Enter main event loop

    //***************************ENDOF BT VISUALIZATION ****************************

}










