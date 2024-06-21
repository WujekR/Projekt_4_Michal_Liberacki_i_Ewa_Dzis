#include "planar_quadrotor_visualizer.h"
#include <cmath>

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor* quadrotor_ptr) : quadrotor_ptr(quadrotor_ptr) {}

void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer>& gRenderer) {
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    float q_x = state[0];
    float q_y = state[1];
    float q_theta = -state[2];

    int image_width = 1280;
    int image_height = 720;

    // Dostosowuje pozycje drona
    float dostosowane_q_x = q_x + image_width / 2;
    float dostosowane_q_y = image_height / 2 - q_y;

    // Wymiary drona (ich polowy)
    int half_width = 60;
    int half_height = 10;

    // Wyliczenie krawedzi drona
    Eigen::Vector2f top_left(-half_width, half_height);
    Eigen::Vector2f top_right(half_width, half_height);
    Eigen::Vector2f bottom_left(-half_width, -half_height);
    Eigen::Vector2f bottom_right(half_width, -half_height);

    Eigen::Matrix2f rotacja;
    rotacja << cos(q_theta), -sin(q_theta), sin(q_theta), cos(q_theta);

    // Rotacja krawedzi wzgledem kata theta
    top_left = rotacja * top_left;
    top_right = rotacja * top_right;
    bottom_left = rotacja * bottom_left;
    bottom_right = rotacja * bottom_right;

    // Przesuniecie krawedzi drona do aktualnej pozycji
    top_left += Eigen::Vector2f(dostosowane_q_x, dostosowane_q_y);
    top_right += Eigen::Vector2f(dostosowane_q_x, dostosowane_q_y);
    bottom_left += Eigen::Vector2f(dostosowane_q_x, dostosowane_q_y);
    bottom_right += Eigen::Vector2f(dostosowane_q_x, dostosowane_q_y);

    // Rysuje wype³niony prostok¹t reprezentuj¹cy drona
    Sint16 vx[4] = { Sint16(top_left.x()), Sint16(top_right.x()), Sint16(bottom_right.x()), Sint16(bottom_left.x()) };
    Sint16 vy[4] = { Sint16(top_left.y()), Sint16(top_right.y()), Sint16(bottom_right.y()), Sint16(bottom_left.y()) };
    filledPolygonColor(gRenderer.get(), vx, vy, 4, 0xFF000000);

    // Pozycja swiatelek
    float green_x = (top_left.x() + bottom_left.x()) / 2;
    float green_y = (top_left.y() + bottom_left.y()) / 2;
    float red_x = (top_right.x() + bottom_right.x()) / 2;
    float red_y = (top_right.y() + bottom_right.y()) / 2;

    // Predkosc smigiel
    Uint32 ticks = SDL_GetTicks();
    float przesuniecie = 8 * sin(ticks / 50.0);

    // Pozycja smigiel
    Eigen::Vector2f lewe_smiglo = Eigen::Vector2f(-50, -25);
    Eigen::Vector2f prawe_smiglo = Eigen::Vector2f(50, -25);

    // Rotacja smigiel
    lewe_smiglo = rotacja * lewe_smiglo;
    prawe_smiglo = rotacja * prawe_smiglo;

    // Przesuniecie smigiel do aktualnej pozycji
    lewe_smiglo += Eigen::Vector2f(dostosowane_q_x, dostosowane_q_y);
    prawe_smiglo += Eigen::Vector2f(dostosowane_q_x, dostosowane_q_y);

    // Ruch smigielek z uwzglednieniem kata theta
    Eigen::Vector2f lewe_smiglo1 = lewe_smiglo + Eigen::Vector2f(przesuniecie * cos(q_theta), przesuniecie * sin(q_theta));
    Eigen::Vector2f lewe_smiglo2 = lewe_smiglo - Eigen::Vector2f(przesuniecie * cos(q_theta), przesuniecie * sin(q_theta));
    Eigen::Vector2f prawe_smiglo1 = prawe_smiglo + Eigen::Vector2f(przesuniecie * cos(q_theta), przesuniecie * sin(q_theta));
    Eigen::Vector2f prawe_smiglo2 = prawe_smiglo - Eigen::Vector2f(przesuniecie * cos(q_theta), przesuniecie * sin(q_theta));

    // Swiatelka
    filledCircleColor(gRenderer.get(), red_x, red_y, 5, 0xFF0000FF);   // Czerwone kolko
    filledCircleColor(gRenderer.get(), green_x, green_y, 5, 0xFF00FF00); // Zielone kolko

    // Smigla
    filledCircleColor(gRenderer.get(), lewe_smiglo1.x(), lewe_smiglo1.y(), 7, 0xFFFF0000);
    filledCircleColor(gRenderer.get(), lewe_smiglo2.x(), lewe_smiglo2.y(), 7, 0xFFFF0000);
    filledCircleColor(gRenderer.get(), prawe_smiglo1.x(), prawe_smiglo1.y(), 7, 0xFFFF0000);
    filledCircleColor(gRenderer.get(), prawe_smiglo2.x(), prawe_smiglo2.y(), 7, 0xFFFF0000);

    // Celownik
    Eigen::VectorXf goal = quadrotor_ptr->GetState() - quadrotor_ptr->GetControlState();
    float wsk_x = goal[0] + image_width / 2;
    float wsk_y = image_height / 2 - goal[1];
    filledCircleColor(gRenderer.get(), wsk_x, wsk_y, 4, 0xFFFF00FF);
    boxColor(gRenderer.get(), wsk_x - 1, wsk_y - 10, wsk_x + 1, wsk_y + 10, 0xFFFF00FF);
    boxColor(gRenderer.get(), wsk_x - 10, wsk_y - 1, wsk_x + 10, wsk_y + 1, 0xFFFF00FF);
}
