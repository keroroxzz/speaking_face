/*
    Robot Face Animation for ROS
    By Brian Tu
    Date : 2023/03/20
*/
#include "face.h"

void RenderScene(void)
{
    #ifdef MEASURE_TIME
    pclock = chrono::high_resolution_clock::now();
    #endif

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(horizontal, -horizontal, -vertical, vertical, 0.0, 100.0f);
    glGetFloatv(GL_PROJECTION_MATRIX, projection);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(
        heading.offset_x,
        heading.offset_y, 
        -20.0, 
        heading.offset_x, 
        heading.offset_y, 
        1.0, 
        sin(heading.rotation), 
        cos(heading.rotation), 
        0.0);
    glGetFloatv(GL_MODELVIEW_MATRIX, model_view);
    m3dMatrixMultiply44(model_view_proj, projection, model_view);

    eye_shader->use();
    eye_shader->setUniform("ModelViewProjMatrix", UNI_MATRIX_4, model_view_proj);
    eye_shader->setUniform("eye_l", UNI_VEC_4, &eye_left);
    eye_shader->setUniform("eye_r", UNI_VEC_4, &eye_right);
    eye_shader->setUniform("mouth", UNI_VEC_4, &mouth_shape);

    eye_shader->setUniform("eye_l_t", UNI_VEC_3, &eye_left.rotation);
    eye_shader->setUniform("eye_r_t", UNI_VEC_3, &eye_right.rotation);
    eye_shader->setUniform("mouth_t", UNI_VEC_3, &mouth_shape.rotation);
    eye_shader->setUniform("eye_distance", UNI_FLOAT_1, &heading.width);

    glPushMatrix();

    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, facerect_vertexbuffer);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glDisableVertexAttribArray(0);

    glPopMatrix();

    // Flush drawing commands
    glutSwapBuffers();

    #ifdef MEASURE_TIME
    printf("rendering time: %f\n", (chrono::high_resolution_clock::now()-pclock).count()/1000000.0);
    pclock = chrono::high_resolution_clock::now();
    #endif
}

//initialization
void SetupRC()
{
    if (!GL_VERSION_2_0 && (!GL_ARB_fragment_shader ||
        !GL_ARB_shader_objects ||
        !GL_ARB_shading_language_100))
    {
        fprintf(stderr, "GLSL extensions not available!\n");
        return;
    }

    // Black background
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    // Misc. state
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_NORMALIZE);

    // Bind the buffer for the face rect
    glGenBuffers(1, &facerect_vertexbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, facerect_vertexbuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(face_rect_buffer_data), face_rect_buffer_data, GL_STATIC_DRAW);
}

void KeyPressFunc(unsigned char key, int x, int y)
{
    if(key==27){
        exit(0);
    }

    else if(key==32){
        glutReshapeWindow(640,480);
    }
}

void ReloadShaders()
{
    if(eye_shader)
        delete eye_shader;

    eye_shader = new Shader(2);
    eye_shader->addFromFile((path+"/shaders/vertex/eye.vs").c_str(), GL_VERTEX_SHADER);
    eye_shader->addFromFile((path+"/shaders/frag/eye.fs").c_str(), GL_FRAGMENT_SHADER);
}

void SpecialKeys(int key, int x, int y)
{
}

void ChangeSize(int w, int h)
{
    windowWidth = w;
    windowHeight = h;

    GLdouble 
        aspect_xy = (GLdouble)windowWidth / (GLdouble)windowHeight,
        aspect_yx = (GLdouble)windowHeight / (GLdouble)windowWidth;

    aspect = max(aspect_yx, aspect_xy);
    vertical = max(1.0, aspect_yx);
    horizontal = max(1.0, aspect_xy);


    glViewport(0, 0, windowWidth, windowHeight);
}

void idle()
{
    ros::spinOnce();
    ReloadShaders();

    eye_left = eye_left*(1.0-face_param.eye_left_speed) + eye_left_target*face_param.eye_left_speed;
    eye_right = eye_right*(1.0-face_param.eye_right_speed) + eye_right_target*face_param.eye_right_speed;
    mouth_shape = mouth_shape*(1.0-face_param.mouth_speed) + mouth_shape_target*face_param.mouth_speed;
    heading = heading*(1.0-face_param.heading_speed) + heading_target*face_param.heading_speed;

    glutPostRedisplay();
    usleep(50);
}

void mousePressed(int x_, int y_)
{
}

void mouseHover(int x_, int y_)
{
    float x = 2.0 * (float)x_ / windowWidth - 1.0;
    float y = 2.0 * (float)y_ / windowHeight - 1.0;
    pmouse[0]=x;
    pmouse[1]=y;
}

void mouse(int button, int state, int x, int y)
{
}
void heading_config_cb(const speaking_face::ShapeConfig::ConstPtr msg){
    copy(heading_target,*msg);
}

void face_config_cb(const speaking_face::FaceConfig::ConstPtr msg){
    copy(eye_left_target,msg->eye_left);
    copy(eye_right_target,msg->eye_right);
    copy(mouth_shape_target,msg->mouth);
}

void face_param_cb(const speaking_face::FaceParam::ConstPtr msg){
    face_param.eye_left_speed = msg->eye_left_speed;
    face_param.eye_right_speed = msg->eye_right_speed;
    face_param.mouth_speed = msg->mouth_speed;
    face_param.heading_speed = msg->heading_speed;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "RobotFace");
    ros::NodeHandle nh;
    ros::Subscriber sub_face, sub_param, sub_heading;
    sub_heading = nh.subscribe("/speaking_face/heading_config", 1, heading_config_cb);
    sub_face = nh.subscribe("/speaking_face/face_config", 1, face_config_cb);
    sub_param = nh.subscribe("/speaking_face/face_param", 1, face_param_cb);

    // Read the parameter
    if (!nh.getParam ("/robot_face/full_screen", full_screen))
        full_screen = false;

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(windowWidth, windowHeight);
    glutCreateWindow("Fight Simulator - alpha");

    eye_shader = new Shader(2);
    eye_shader->addFromFile("/home/rtu/catkin_clip_topo/src/speaking_face/shaders/vertex/eye.vs", GL_VERTEX_SHADER);
    eye_shader->addFromFile("/home/rtu/catkin_clip_topo/src/speaking_face/shaders/frag/eye.fs", GL_FRAGMENT_SHADER);

    glutReshapeFunc(ChangeSize);
    glutKeyboardFunc(KeyPressFunc);
    glutDisplayFunc(RenderScene);
    glutIdleFunc(idle);
    // glutMotionFunc(mousePressed);
    // glutPassiveMotionFunc(mouseHover);
    // glutMouseFunc(mouse);
    if(full_screen)
        glutFullScreen();

    SetupRC();
    glutMainLoop();

    return 0;
}
