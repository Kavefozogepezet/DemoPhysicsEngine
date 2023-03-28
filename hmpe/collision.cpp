//
// Created by User on 15/03/2023.
//

#include <iostream>

#include "hmpe/utils.hpp"
#include "hmmath/math.hpp"
#include "hmpe/body.hpp"
#include "hmpe/collision.hpp"

HMPE_START

ContactInfo ContactInfo::of(BodyRef b1, BodyRef b2) {
    ContactInfo info {};
    if(b2->type == BodyType::KINEMATIC) {
        info.body1 = b1;
        info.body2 = b2;
    } else {
        info.body1 = b2;
        info.body2 = b1;
    }
    return info;
}

void Manifold::addContact(ContactInfo &info, UpdateID now) {
    last = now;
    for(auto it = contacts.begin(); it != contacts.end();) {
        float l1 = Math::length(it->point1 - info.point1);
        float l2 = Math::length(it->point2 - info.point2);
        bool tooClose = l1 < 0.1f || l2 < 0.1f;

        Vec2 oldPos1 = info.body1->position + it->localPoint1;
        Vec2 oldPos2 = info.body2->position + it->localPoint2;
        bool r1 = !Math::floatEq(Math::length(oldPos1 - it->point1), 0.0f, 50000.0f);
        bool r2 = !Math::floatEq(Math::length(oldPos2 - it->point2), 0.0f, 50000.0f);

        if(tooClose || r1 || r2) it = contacts.erase(it);
        else it++;
    }
    contacts.push_front(info);
    if(contacts.size() > 2)
        contacts.pop_back();
}

bool Manifold::isOutdated(UpdateID now) const {
    return now != last;
}

struct SupportPoint : public Vec2 {
    Vec2 pointOnB1, pointOnB2;

    SupportPoint() = default;

    SupportPoint(BodyRef b1, BodyRef b2, Vec2 normal) :
            Vec2{},
            pointOnB1(b1->support(normal)),
            pointOnB2(b2->support(-normal))
    {
        (Vec2&)(*this) = pointOnB1 - pointOnB2;
    }
};

class Simplex {
public:
    static bool init(BodyRef b1, BodyRef b2, Simplex& simplex) {
        Vec2 delta = b1->position - b2->position;
        Vec2 n = delta == Vec2 { 0.0f, 0.0f } ?
                 Vec2 { 1.0f, 0.0f } :
                 Math::normalize(delta); //TODO delta

        simplex[0] = SupportPoint(b1, b2, n);
        simplex[1] = SupportPoint(b1, b2, Math::normalize(-simplex[0]));
        if(!Math::isOpposing(simplex[0], simplex[1]))
            return false;
        return true;
    }

    SupportPoint& operator[](size_t idx) {
        return points[idx];
    }

    const SupportPoint& operator[](size_t idx) const {
        return points[idx];
    }

    void nextPoint(const SupportPoint& point) {
        Vec2
                n1 = Math::normalToOrigin(points[0], point),
                n2 = Math::normalToOrigin(points[1], point),
                p0p1 = points[1] - points[0];


        if(Math::dot(n1, p0p1) < 0.0f) {
            points[2] = points[1];
            points[1] = point;
        } else if(Math::dot(n2, -p0p1) < 0.0f) {
            points[2] = points[0];
            points[0] = point;
        } else {
            points[2] = point;
            contains = true;
        }
    }

    [[nodiscard]]
    bool isPoint(Vec2 p) const {
        for(auto& point : points) {
            float delta = Math::length(point - p);
            if(Math::floatEq(delta, 0.0f))
                return true;
        }
        return false;
    }

    [[nodiscard]]
    bool containsOrigin() const {
        return contains;
    }
private:
    SupportPoint points[3] {};
    bool contains = false;
};

class PolytopeEdge {
public:
    explicit PolytopeEdge(const SupportPoint &point) :
        point(point),
        nextEdge(this)
    {}

    PolytopeEdge(const SupportPoint &point, PolytopeEdge *nextEdge) :
        point(point),
        nextEdge(nextEdge)
    {}

    [[nodiscard]]
    const SupportPoint &p1() const {
        return point;
    }

    [[nodiscard]]
    const SupportPoint &p2() const {
        return nextEdge->p1();
    }

    PolytopeEdge *next() {
        return nextEdge;
    }

    void insert(const SupportPoint &p) {
        auto *temp = new PolytopeEdge(p, nextEdge);
        nextEdge = temp;
    }

    [[nodiscard]]
    bool isPoint(const Vec2& p) const {
        Vec2 n = Math::normalToOrigin(p1(), p2());
        Vec2 p1p = p - p1();
        return Math::floatEq(Math::dot(n, p1p), 0.0f);
    }

    [[nodiscard]]
    float projectOrigin() const {
        Vec2 side = p2() - p1();
        float dot = Math::dot(-p1(), Math::normalize(side));
        return  dot / Math::length(side);
    }
private:
    SupportPoint point;
    PolytopeEdge *nextEdge;
};

static std::ostream& operator << (std::ostream& stream, const Vec2& vec) {
    return stream << '(' << vec[0] << ',' << vec[1] << ')';
}

struct Polytope {
    PolytopeEdge *edge;

    explicit Polytope(Simplex simplex) {
        edge = new PolytopeEdge(simplex[0]);
        edge->insert(simplex[1]);
        edge->insert(simplex[2]);
    }

    ~Polytope() {
        PolytopeEdge *temp;
        auto *next = edge;
        do {
            temp = next;
            next = temp->next();
            delete temp;
        } while (next != edge);
    }

    void findClosestEdge(PolytopeEdge*& closest, Vec2& normal, float& depth) const {
        depth = std::numeric_limits<float>::max();
        auto* current = edge;
        do {
            Vec2 n = -Math::normalToOrigin(current->p1(), current->p2());
            float d = Math::dot(n, current->p1());

            if(d < depth) {
                float p = current->projectOrigin();
                if(p >= 0.0f && p < 1.0f) {
                    closest = current;
                    depth = d;
                    normal = n;
                }
            }
            current = current->next();
        } while(current != edge);
    }
};

static bool gjk(BodyRef b1, BodyRef b2, Simplex& simplex) {
    if(!Simplex::init(b1, b2, simplex))
        return false;

    do {
        Vec2 n = Math::normalToOrigin(simplex[0], simplex[1]);
        SupportPoint p(b1, b2, n);
        if(Math::isOpposing(p, n) || simplex.isPoint(p))
            return false;
        simplex.nextPoint(p);
    } while(!simplex.containsOrigin());
    return true;
}

static void epa(Polytope& polytope, ContactInfo& info) {
    BodyRef b1 = info.body1;
    BodyRef b2 = info.body2;
    PolytopeEdge* closest = nullptr;
    SupportPoint currentSupport {};

    size_t i = 0;
    do {
        if(closest)
            closest->insert(currentSupport);
        polytope.findClosestEdge(closest, info.normal, info.depth);
        currentSupport = SupportPoint(b1, b2, info.normal);
        i++;
    } while(!closest->isPoint(currentSupport));

    float ratio = closest->projectOrigin();

    info.point1 = Math::lerp(closest->p1().pointOnB1, closest->p2().pointOnB1, ratio);
    info.localPoint1 = info.point1 - b1 ->position;

    info.point2 = Math::lerp(closest->p1().pointOnB2, closest->p2().pointOnB2, ratio);
    info.localPoint2 = info.point2 - b2->position;

    info.normal = -Math::normalToOrigin(closest->p1(), closest->p2());
}

void collide(ContactInfo& info) {
    if(!(info.body1 && info.body2))
        return;

    if(info.body1->type == BodyType::KINEMATIC)
        return;

    Simplex simplex;
    info.collision = gjk(info.body1, info.body2, simplex);
    if(info.collision) {
        Polytope polytope(simplex);
        epa(polytope, info);

        // TODO This is a hotfix for epa finding the contact points on opposing ends of the objects causing an explosion
        float l = Math::length(info.point1 - info.point2);
        if(l > 0.5f)
            info.collision = false;
    }
}

bool overlap(BodyRef body1, BodyRef body2) {
    if(!(body1 && body2))
        return false;

    Simplex simplex;
    return gjk(body1, body2, simplex);
}

HMPE_END
