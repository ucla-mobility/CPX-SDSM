//
// Generated file, do not edit! Created by opp_msgtool 6.0 from messages/SdsmPayload.msg.
//

// Disable warnings about unused variables, empty switch stmts, etc:
#ifdef _MSC_VER
#  pragma warning(disable:4101)
#  pragma warning(disable:4065)
#endif

#if defined(__clang__)
#  pragma clang diagnostic ignored "-Wshadow"
#  pragma clang diagnostic ignored "-Wconversion"
#  pragma clang diagnostic ignored "-Wunused-parameter"
#  pragma clang diagnostic ignored "-Wc++98-compat"
#  pragma clang diagnostic ignored "-Wunreachable-code-break"
#  pragma clang diagnostic ignored "-Wold-style-cast"
#elif defined(__GNUC__)
#  pragma GCC diagnostic ignored "-Wshadow"
#  pragma GCC diagnostic ignored "-Wconversion"
#  pragma GCC diagnostic ignored "-Wunused-parameter"
#  pragma GCC diagnostic ignored "-Wold-style-cast"
#  pragma GCC diagnostic ignored "-Wsuggest-attribute=noreturn"
#  pragma GCC diagnostic ignored "-Wfloat-conversion"
#endif

#include <iostream>
#include <sstream>
#include <memory>
#include <type_traits>
#include "SdsmPayload_m.h"

namespace omnetpp {

// Template pack/unpack rules. They are declared *after* a1l type-specific pack functions for multiple reasons.
// They are in the omnetpp namespace, to allow them to be found by argument-dependent lookup via the cCommBuffer argument

// Packing/unpacking an std::vector
template<typename T, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::vector<T,A>& v)
{
    int n = v.size();
    doParsimPacking(buffer, n);
    for (int i = 0; i < n; i++)
        doParsimPacking(buffer, v[i]);
}

template<typename T, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::vector<T,A>& v)
{
    int n;
    doParsimUnpacking(buffer, n);
    v.resize(n);
    for (int i = 0; i < n; i++)
        doParsimUnpacking(buffer, v[i]);
}

// Packing/unpacking an std::list
template<typename T, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::list<T,A>& l)
{
    doParsimPacking(buffer, (int)l.size());
    for (typename std::list<T,A>::const_iterator it = l.begin(); it != l.end(); ++it)
        doParsimPacking(buffer, (T&)*it);
}

template<typename T, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::list<T,A>& l)
{
    int n;
    doParsimUnpacking(buffer, n);
    for (int i = 0; i < n; i++) {
        l.push_back(T());
        doParsimUnpacking(buffer, l.back());
    }
}

// Packing/unpacking an std::set
template<typename T, typename Tr, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::set<T,Tr,A>& s)
{
    doParsimPacking(buffer, (int)s.size());
    for (typename std::set<T,Tr,A>::const_iterator it = s.begin(); it != s.end(); ++it)
        doParsimPacking(buffer, *it);
}

template<typename T, typename Tr, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::set<T,Tr,A>& s)
{
    int n;
    doParsimUnpacking(buffer, n);
    for (int i = 0; i < n; i++) {
        T x;
        doParsimUnpacking(buffer, x);
        s.insert(x);
    }
}

// Packing/unpacking an std::map
template<typename K, typename V, typename Tr, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::map<K,V,Tr,A>& m)
{
    doParsimPacking(buffer, (int)m.size());
    for (typename std::map<K,V,Tr,A>::const_iterator it = m.begin(); it != m.end(); ++it) {
        doParsimPacking(buffer, it->first);
        doParsimPacking(buffer, it->second);
    }
}

template<typename K, typename V, typename Tr, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::map<K,V,Tr,A>& m)
{
    int n;
    doParsimUnpacking(buffer, n);
    for (int i = 0; i < n; i++) {
        K k; V v;
        doParsimUnpacking(buffer, k);
        doParsimUnpacking(buffer, v);
        m[k] = v;
    }
}

// Default pack/unpack function for arrays
template<typename T>
void doParsimArrayPacking(omnetpp::cCommBuffer *b, const T *t, int n)
{
    for (int i = 0; i < n; i++)
        doParsimPacking(b, t[i]);
}

template<typename T>
void doParsimArrayUnpacking(omnetpp::cCommBuffer *b, T *t, int n)
{
    for (int i = 0; i < n; i++)
        doParsimUnpacking(b, t[i]);
}

// Default rule to prevent compiler from choosing base class' doParsimPacking() function
template<typename T>
void doParsimPacking(omnetpp::cCommBuffer *, const T& t)
{
    throw omnetpp::cRuntimeError("Parsim error: No doParsimPacking() function for type %s", omnetpp::opp_typename(typeid(t)));
}

template<typename T>
void doParsimUnpacking(omnetpp::cCommBuffer *, T& t)
{
    throw omnetpp::cRuntimeError("Parsim error: No doParsimUnpacking() function for type %s", omnetpp::opp_typename(typeid(t)));
}

}  // namespace omnetpp

namespace veins_ros_v2v {
namespace messages {

Register_Class(SdsmPayload)

SdsmPayload::SdsmPayload(const char *name, short kind) : ::omnetpp::cPacket(name, kind)
{
}

SdsmPayload::SdsmPayload(const SdsmPayload& other) : ::omnetpp::cPacket(other)
{
    copy(other);
}

SdsmPayload::~SdsmPayload()
{
}

SdsmPayload& SdsmPayload::operator=(const SdsmPayload& other)
{
    if (this == &other) return *this;
    ::omnetpp::cPacket::operator=(other);
    copy(other);
    return *this;
}

void SdsmPayload::copy(const SdsmPayload& other)
{
    this->payload = other.payload;
    this->senderNodeIndex = other.senderNodeIndex;
    this->senderX = other.senderX;
    this->senderY = other.senderY;
    this->senderSpeed = other.senderSpeed;
    this->senderHeading = other.senderHeading;
    this->sendTimestamp = other.sendTimestamp;
    this->message_id = other.message_id;
    this->msg_cnt = other.msg_cnt;
    for (size_t i = 0; i < 4; i++) {
        this->source_id[i] = other.source_id[i];
    }
    this->equipment_type = other.equipment_type;
    this->ref_pos_x = other.ref_pos_x;
    this->ref_pos_y = other.ref_pos_y;
    this->ref_pos_z = other.ref_pos_z;
    this->sdsm_day = other.sdsm_day;
    this->sdsm_time_of_day_ms = other.sdsm_time_of_day_ms;
    this->numObjects = other.numObjects;
    for (size_t i = 0; i < 16; i++) {
        this->obj_type[i] = other.obj_type[i];
    }
    for (size_t i = 0; i < 16; i++) {
        this->object_id[i] = other.object_id[i];
    }
    for (size_t i = 0; i < 16; i++) {
        this->offset_x[i] = other.offset_x[i];
    }
    for (size_t i = 0; i < 16; i++) {
        this->offset_y[i] = other.offset_y[i];
    }
    for (size_t i = 0; i < 16; i++) {
        this->offset_z[i] = other.offset_z[i];
    }
    for (size_t i = 0; i < 16; i++) {
        this->obj_speed[i] = other.obj_speed[i];
    }
    for (size_t i = 0; i < 16; i++) {
        this->obj_heading[i] = other.obj_heading[i];
    }
}

void SdsmPayload::parsimPack(omnetpp::cCommBuffer *b) const
{
    ::omnetpp::cPacket::parsimPack(b);
    doParsimPacking(b,this->payload);
    doParsimPacking(b,this->senderNodeIndex);
    doParsimPacking(b,this->senderX);
    doParsimPacking(b,this->senderY);
    doParsimPacking(b,this->senderSpeed);
    doParsimPacking(b,this->senderHeading);
    doParsimPacking(b,this->sendTimestamp);
    doParsimPacking(b,this->message_id);
    doParsimPacking(b,this->msg_cnt);
    doParsimArrayPacking(b,this->source_id,4);
    doParsimPacking(b,this->equipment_type);
    doParsimPacking(b,this->ref_pos_x);
    doParsimPacking(b,this->ref_pos_y);
    doParsimPacking(b,this->ref_pos_z);
    doParsimPacking(b,this->sdsm_day);
    doParsimPacking(b,this->sdsm_time_of_day_ms);
    doParsimPacking(b,this->numObjects);
    doParsimArrayPacking(b,this->obj_type,16);
    doParsimArrayPacking(b,this->object_id,16);
    doParsimArrayPacking(b,this->offset_x,16);
    doParsimArrayPacking(b,this->offset_y,16);
    doParsimArrayPacking(b,this->offset_z,16);
    doParsimArrayPacking(b,this->obj_speed,16);
    doParsimArrayPacking(b,this->obj_heading,16);
}

void SdsmPayload::parsimUnpack(omnetpp::cCommBuffer *b)
{
    ::omnetpp::cPacket::parsimUnpack(b);
    doParsimUnpacking(b,this->payload);
    doParsimUnpacking(b,this->senderNodeIndex);
    doParsimUnpacking(b,this->senderX);
    doParsimUnpacking(b,this->senderY);
    doParsimUnpacking(b,this->senderSpeed);
    doParsimUnpacking(b,this->senderHeading);
    doParsimUnpacking(b,this->sendTimestamp);
    doParsimUnpacking(b,this->message_id);
    doParsimUnpacking(b,this->msg_cnt);
    doParsimArrayUnpacking(b,this->source_id,4);
    doParsimUnpacking(b,this->equipment_type);
    doParsimUnpacking(b,this->ref_pos_x);
    doParsimUnpacking(b,this->ref_pos_y);
    doParsimUnpacking(b,this->ref_pos_z);
    doParsimUnpacking(b,this->sdsm_day);
    doParsimUnpacking(b,this->sdsm_time_of_day_ms);
    doParsimUnpacking(b,this->numObjects);
    doParsimArrayUnpacking(b,this->obj_type,16);
    doParsimArrayUnpacking(b,this->object_id,16);
    doParsimArrayUnpacking(b,this->offset_x,16);
    doParsimArrayUnpacking(b,this->offset_y,16);
    doParsimArrayUnpacking(b,this->offset_z,16);
    doParsimArrayUnpacking(b,this->obj_speed,16);
    doParsimArrayUnpacking(b,this->obj_heading,16);
}

const char * SdsmPayload::getPayload() const
{
    return this->payload.c_str();
}

void SdsmPayload::setPayload(const char * payload)
{
    this->payload = payload;
}

int SdsmPayload::getSenderNodeIndex() const
{
    return this->senderNodeIndex;
}

void SdsmPayload::setSenderNodeIndex(int senderNodeIndex)
{
    this->senderNodeIndex = senderNodeIndex;
}

double SdsmPayload::getSenderX() const
{
    return this->senderX;
}

void SdsmPayload::setSenderX(double senderX)
{
    this->senderX = senderX;
}

double SdsmPayload::getSenderY() const
{
    return this->senderY;
}

void SdsmPayload::setSenderY(double senderY)
{
    this->senderY = senderY;
}

double SdsmPayload::getSenderSpeed() const
{
    return this->senderSpeed;
}

void SdsmPayload::setSenderSpeed(double senderSpeed)
{
    this->senderSpeed = senderSpeed;
}

double SdsmPayload::getSenderHeading() const
{
    return this->senderHeading;
}

void SdsmPayload::setSenderHeading(double senderHeading)
{
    this->senderHeading = senderHeading;
}

double SdsmPayload::getSendTimestamp() const
{
    return this->sendTimestamp;
}

void SdsmPayload::setSendTimestamp(double sendTimestamp)
{
    this->sendTimestamp = sendTimestamp;
}

long SdsmPayload::getMessage_id() const
{
    return this->message_id;
}

void SdsmPayload::setMessage_id(long message_id)
{
    this->message_id = message_id;
}

int SdsmPayload::getMsg_cnt() const
{
    return this->msg_cnt;
}

void SdsmPayload::setMsg_cnt(int msg_cnt)
{
    this->msg_cnt = msg_cnt;
}

size_t SdsmPayload::getSource_idArraySize() const
{
    return 4;
}

int SdsmPayload::getSource_id(size_t k) const
{
    if (k >= 4) throw omnetpp::cRuntimeError("Array of size %lu indexed by %lu", (unsigned long)4, (unsigned long)k);
    return this->source_id[k];
}

void SdsmPayload::setSource_id(size_t k, int source_id)
{
    if (k >= 4) throw omnetpp::cRuntimeError("Array of size %lu indexed by %lu", (unsigned long)4, (unsigned long)k);
    this->source_id[k] = source_id;
}

int SdsmPayload::getEquipment_type() const
{
    return this->equipment_type;
}

void SdsmPayload::setEquipment_type(int equipment_type)
{
    this->equipment_type = equipment_type;
}

double SdsmPayload::getRef_pos_x() const
{
    return this->ref_pos_x;
}

void SdsmPayload::setRef_pos_x(double ref_pos_x)
{
    this->ref_pos_x = ref_pos_x;
}

double SdsmPayload::getRef_pos_y() const
{
    return this->ref_pos_y;
}

void SdsmPayload::setRef_pos_y(double ref_pos_y)
{
    this->ref_pos_y = ref_pos_y;
}

double SdsmPayload::getRef_pos_z() const
{
    return this->ref_pos_z;
}

void SdsmPayload::setRef_pos_z(double ref_pos_z)
{
    this->ref_pos_z = ref_pos_z;
}

int SdsmPayload::getSdsm_day() const
{
    return this->sdsm_day;
}

void SdsmPayload::setSdsm_day(int sdsm_day)
{
    this->sdsm_day = sdsm_day;
}

long SdsmPayload::getSdsm_time_of_day_ms() const
{
    return this->sdsm_time_of_day_ms;
}

void SdsmPayload::setSdsm_time_of_day_ms(long sdsm_time_of_day_ms)
{
    this->sdsm_time_of_day_ms = sdsm_time_of_day_ms;
}

int SdsmPayload::getNumObjects() const
{
    return this->numObjects;
}

void SdsmPayload::setNumObjects(int numObjects)
{
    this->numObjects = numObjects;
}

size_t SdsmPayload::getObj_typeArraySize() const
{
    return 16;
}

int SdsmPayload::getObj_type(size_t k) const
{
    if (k >= 16) throw omnetpp::cRuntimeError("Array of size %lu indexed by %lu", (unsigned long)16, (unsigned long)k);
    return this->obj_type[k];
}

void SdsmPayload::setObj_type(size_t k, int obj_type)
{
    if (k >= 16) throw omnetpp::cRuntimeError("Array of size %lu indexed by %lu", (unsigned long)16, (unsigned long)k);
    this->obj_type[k] = obj_type;
}

size_t SdsmPayload::getObject_idArraySize() const
{
    return 16;
}

int SdsmPayload::getObject_id(size_t k) const
{
    if (k >= 16) throw omnetpp::cRuntimeError("Array of size %lu indexed by %lu", (unsigned long)16, (unsigned long)k);
    return this->object_id[k];
}

void SdsmPayload::setObject_id(size_t k, int object_id)
{
    if (k >= 16) throw omnetpp::cRuntimeError("Array of size %lu indexed by %lu", (unsigned long)16, (unsigned long)k);
    this->object_id[k] = object_id;
}

size_t SdsmPayload::getOffset_xArraySize() const
{
    return 16;
}

int SdsmPayload::getOffset_x(size_t k) const
{
    if (k >= 16) throw omnetpp::cRuntimeError("Array of size %lu indexed by %lu", (unsigned long)16, (unsigned long)k);
    return this->offset_x[k];
}

void SdsmPayload::setOffset_x(size_t k, int offset_x)
{
    if (k >= 16) throw omnetpp::cRuntimeError("Array of size %lu indexed by %lu", (unsigned long)16, (unsigned long)k);
    this->offset_x[k] = offset_x;
}

size_t SdsmPayload::getOffset_yArraySize() const
{
    return 16;
}

int SdsmPayload::getOffset_y(size_t k) const
{
    if (k >= 16) throw omnetpp::cRuntimeError("Array of size %lu indexed by %lu", (unsigned long)16, (unsigned long)k);
    return this->offset_y[k];
}

void SdsmPayload::setOffset_y(size_t k, int offset_y)
{
    if (k >= 16) throw omnetpp::cRuntimeError("Array of size %lu indexed by %lu", (unsigned long)16, (unsigned long)k);
    this->offset_y[k] = offset_y;
}

size_t SdsmPayload::getOffset_zArraySize() const
{
    return 16;
}

int SdsmPayload::getOffset_z(size_t k) const
{
    if (k >= 16) throw omnetpp::cRuntimeError("Array of size %lu indexed by %lu", (unsigned long)16, (unsigned long)k);
    return this->offset_z[k];
}

void SdsmPayload::setOffset_z(size_t k, int offset_z)
{
    if (k >= 16) throw omnetpp::cRuntimeError("Array of size %lu indexed by %lu", (unsigned long)16, (unsigned long)k);
    this->offset_z[k] = offset_z;
}

size_t SdsmPayload::getObj_speedArraySize() const
{
    return 16;
}

int SdsmPayload::getObj_speed(size_t k) const
{
    if (k >= 16) throw omnetpp::cRuntimeError("Array of size %lu indexed by %lu", (unsigned long)16, (unsigned long)k);
    return this->obj_speed[k];
}

void SdsmPayload::setObj_speed(size_t k, int obj_speed)
{
    if (k >= 16) throw omnetpp::cRuntimeError("Array of size %lu indexed by %lu", (unsigned long)16, (unsigned long)k);
    this->obj_speed[k] = obj_speed;
}

size_t SdsmPayload::getObj_headingArraySize() const
{
    return 16;
}

int SdsmPayload::getObj_heading(size_t k) const
{
    if (k >= 16) throw omnetpp::cRuntimeError("Array of size %lu indexed by %lu", (unsigned long)16, (unsigned long)k);
    return this->obj_heading[k];
}

void SdsmPayload::setObj_heading(size_t k, int obj_heading)
{
    if (k >= 16) throw omnetpp::cRuntimeError("Array of size %lu indexed by %lu", (unsigned long)16, (unsigned long)k);
    this->obj_heading[k] = obj_heading;
}

class SdsmPayloadDescriptor : public omnetpp::cClassDescriptor
{
  private:
    mutable const char **propertyNames;
    enum FieldConstants {
        FIELD_payload,
        FIELD_senderNodeIndex,
        FIELD_senderX,
        FIELD_senderY,
        FIELD_senderSpeed,
        FIELD_senderHeading,
        FIELD_sendTimestamp,
        FIELD_message_id,
        FIELD_msg_cnt,
        FIELD_source_id,
        FIELD_equipment_type,
        FIELD_ref_pos_x,
        FIELD_ref_pos_y,
        FIELD_ref_pos_z,
        FIELD_sdsm_day,
        FIELD_sdsm_time_of_day_ms,
        FIELD_numObjects,
        FIELD_obj_type,
        FIELD_object_id,
        FIELD_offset_x,
        FIELD_offset_y,
        FIELD_offset_z,
        FIELD_obj_speed,
        FIELD_obj_heading,
    };
  public:
    SdsmPayloadDescriptor();
    virtual ~SdsmPayloadDescriptor();

    virtual bool doesSupport(omnetpp::cObject *obj) const override;
    virtual const char **getPropertyNames() const override;
    virtual const char *getProperty(const char *propertyName) const override;
    virtual int getFieldCount() const override;
    virtual const char *getFieldName(int field) const override;
    virtual int findField(const char *fieldName) const override;
    virtual unsigned int getFieldTypeFlags(int field) const override;
    virtual const char *getFieldTypeString(int field) const override;
    virtual const char **getFieldPropertyNames(int field) const override;
    virtual const char *getFieldProperty(int field, const char *propertyName) const override;
    virtual int getFieldArraySize(omnetpp::any_ptr object, int field) const override;
    virtual void setFieldArraySize(omnetpp::any_ptr object, int field, int size) const override;

    virtual const char *getFieldDynamicTypeString(omnetpp::any_ptr object, int field, int i) const override;
    virtual std::string getFieldValueAsString(omnetpp::any_ptr object, int field, int i) const override;
    virtual void setFieldValueAsString(omnetpp::any_ptr object, int field, int i, const char *value) const override;
    virtual omnetpp::cValue getFieldValue(omnetpp::any_ptr object, int field, int i) const override;
    virtual void setFieldValue(omnetpp::any_ptr object, int field, int i, const omnetpp::cValue& value) const override;

    virtual const char *getFieldStructName(int field) const override;
    virtual omnetpp::any_ptr getFieldStructValuePointer(omnetpp::any_ptr object, int field, int i) const override;
    virtual void setFieldStructValuePointer(omnetpp::any_ptr object, int field, int i, omnetpp::any_ptr ptr) const override;
};

Register_ClassDescriptor(SdsmPayloadDescriptor)

SdsmPayloadDescriptor::SdsmPayloadDescriptor() : omnetpp::cClassDescriptor(omnetpp::opp_typename(typeid(veins_ros_v2v::messages::SdsmPayload)), "omnetpp::cPacket")
{
    propertyNames = nullptr;
}

SdsmPayloadDescriptor::~SdsmPayloadDescriptor()
{
    delete[] propertyNames;
}

bool SdsmPayloadDescriptor::doesSupport(omnetpp::cObject *obj) const
{
    return dynamic_cast<SdsmPayload *>(obj)!=nullptr;
}

const char **SdsmPayloadDescriptor::getPropertyNames() const
{
    if (!propertyNames) {
        static const char *names[] = {  nullptr };
        omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
        const char **baseNames = base ? base->getPropertyNames() : nullptr;
        propertyNames = mergeLists(baseNames, names);
    }
    return propertyNames;
}

const char *SdsmPayloadDescriptor::getProperty(const char *propertyName) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    return base ? base->getProperty(propertyName) : nullptr;
}

int SdsmPayloadDescriptor::getFieldCount() const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    return base ? 24+base->getFieldCount() : 24;
}

unsigned int SdsmPayloadDescriptor::getFieldTypeFlags(int field) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldTypeFlags(field);
        field -= base->getFieldCount();
    }
    static unsigned int fieldTypeFlags[] = {
        FD_ISEDITABLE,    // FIELD_payload
        FD_ISEDITABLE,    // FIELD_senderNodeIndex
        FD_ISEDITABLE,    // FIELD_senderX
        FD_ISEDITABLE,    // FIELD_senderY
        FD_ISEDITABLE,    // FIELD_senderSpeed
        FD_ISEDITABLE,    // FIELD_senderHeading
        FD_ISEDITABLE,    // FIELD_sendTimestamp
        FD_ISEDITABLE,    // FIELD_message_id
        FD_ISEDITABLE,    // FIELD_msg_cnt
        FD_ISARRAY | FD_ISEDITABLE,    // FIELD_source_id
        FD_ISEDITABLE,    // FIELD_equipment_type
        FD_ISEDITABLE,    // FIELD_ref_pos_x
        FD_ISEDITABLE,    // FIELD_ref_pos_y
        FD_ISEDITABLE,    // FIELD_ref_pos_z
        FD_ISEDITABLE,    // FIELD_sdsm_day
        FD_ISEDITABLE,    // FIELD_sdsm_time_of_day_ms
        FD_ISEDITABLE,    // FIELD_numObjects
        FD_ISARRAY | FD_ISEDITABLE,    // FIELD_obj_type
        FD_ISARRAY | FD_ISEDITABLE,    // FIELD_object_id
        FD_ISARRAY | FD_ISEDITABLE,    // FIELD_offset_x
        FD_ISARRAY | FD_ISEDITABLE,    // FIELD_offset_y
        FD_ISARRAY | FD_ISEDITABLE,    // FIELD_offset_z
        FD_ISARRAY | FD_ISEDITABLE,    // FIELD_obj_speed
        FD_ISARRAY | FD_ISEDITABLE,    // FIELD_obj_heading
    };
    return (field >= 0 && field < 24) ? fieldTypeFlags[field] : 0;
}

const char *SdsmPayloadDescriptor::getFieldName(int field) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldName(field);
        field -= base->getFieldCount();
    }
    static const char *fieldNames[] = {
        "payload",
        "senderNodeIndex",
        "senderX",
        "senderY",
        "senderSpeed",
        "senderHeading",
        "sendTimestamp",
        "message_id",
        "msg_cnt",
        "source_id",
        "equipment_type",
        "ref_pos_x",
        "ref_pos_y",
        "ref_pos_z",
        "sdsm_day",
        "sdsm_time_of_day_ms",
        "numObjects",
        "obj_type",
        "object_id",
        "offset_x",
        "offset_y",
        "offset_z",
        "obj_speed",
        "obj_heading",
    };
    return (field >= 0 && field < 24) ? fieldNames[field] : nullptr;
}

int SdsmPayloadDescriptor::findField(const char *fieldName) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    int baseIndex = base ? base->getFieldCount() : 0;
    if (strcmp(fieldName, "payload") == 0) return baseIndex + 0;
    if (strcmp(fieldName, "senderNodeIndex") == 0) return baseIndex + 1;
    if (strcmp(fieldName, "senderX") == 0) return baseIndex + 2;
    if (strcmp(fieldName, "senderY") == 0) return baseIndex + 3;
    if (strcmp(fieldName, "senderSpeed") == 0) return baseIndex + 4;
    if (strcmp(fieldName, "senderHeading") == 0) return baseIndex + 5;
    if (strcmp(fieldName, "sendTimestamp") == 0) return baseIndex + 6;
    if (strcmp(fieldName, "message_id") == 0) return baseIndex + 7;
    if (strcmp(fieldName, "msg_cnt") == 0) return baseIndex + 8;
    if (strcmp(fieldName, "source_id") == 0) return baseIndex + 9;
    if (strcmp(fieldName, "equipment_type") == 0) return baseIndex + 10;
    if (strcmp(fieldName, "ref_pos_x") == 0) return baseIndex + 11;
    if (strcmp(fieldName, "ref_pos_y") == 0) return baseIndex + 12;
    if (strcmp(fieldName, "ref_pos_z") == 0) return baseIndex + 13;
    if (strcmp(fieldName, "sdsm_day") == 0) return baseIndex + 14;
    if (strcmp(fieldName, "sdsm_time_of_day_ms") == 0) return baseIndex + 15;
    if (strcmp(fieldName, "numObjects") == 0) return baseIndex + 16;
    if (strcmp(fieldName, "obj_type") == 0) return baseIndex + 17;
    if (strcmp(fieldName, "object_id") == 0) return baseIndex + 18;
    if (strcmp(fieldName, "offset_x") == 0) return baseIndex + 19;
    if (strcmp(fieldName, "offset_y") == 0) return baseIndex + 20;
    if (strcmp(fieldName, "offset_z") == 0) return baseIndex + 21;
    if (strcmp(fieldName, "obj_speed") == 0) return baseIndex + 22;
    if (strcmp(fieldName, "obj_heading") == 0) return baseIndex + 23;
    return base ? base->findField(fieldName) : -1;
}

const char *SdsmPayloadDescriptor::getFieldTypeString(int field) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldTypeString(field);
        field -= base->getFieldCount();
    }
    static const char *fieldTypeStrings[] = {
        "string",    // FIELD_payload
        "int",    // FIELD_senderNodeIndex
        "double",    // FIELD_senderX
        "double",    // FIELD_senderY
        "double",    // FIELD_senderSpeed
        "double",    // FIELD_senderHeading
        "double",    // FIELD_sendTimestamp
        "long",    // FIELD_message_id
        "int",    // FIELD_msg_cnt
        "int",    // FIELD_source_id
        "int",    // FIELD_equipment_type
        "double",    // FIELD_ref_pos_x
        "double",    // FIELD_ref_pos_y
        "double",    // FIELD_ref_pos_z
        "int",    // FIELD_sdsm_day
        "long",    // FIELD_sdsm_time_of_day_ms
        "int",    // FIELD_numObjects
        "int",    // FIELD_obj_type
        "int",    // FIELD_object_id
        "int",    // FIELD_offset_x
        "int",    // FIELD_offset_y
        "int",    // FIELD_offset_z
        "int",    // FIELD_obj_speed
        "int",    // FIELD_obj_heading
    };
    return (field >= 0 && field < 24) ? fieldTypeStrings[field] : nullptr;
}

const char **SdsmPayloadDescriptor::getFieldPropertyNames(int field) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldPropertyNames(field);
        field -= base->getFieldCount();
    }
    switch (field) {
        default: return nullptr;
    }
}

const char *SdsmPayloadDescriptor::getFieldProperty(int field, const char *propertyName) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldProperty(field, propertyName);
        field -= base->getFieldCount();
    }
    switch (field) {
        default: return nullptr;
    }
}

int SdsmPayloadDescriptor::getFieldArraySize(omnetpp::any_ptr object, int field) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldArraySize(object, field);
        field -= base->getFieldCount();
    }
    SdsmPayload *pp = omnetpp::fromAnyPtr<SdsmPayload>(object); (void)pp;
    switch (field) {
        case FIELD_source_id: return 4;
        case FIELD_obj_type: return 16;
        case FIELD_object_id: return 16;
        case FIELD_offset_x: return 16;
        case FIELD_offset_y: return 16;
        case FIELD_offset_z: return 16;
        case FIELD_obj_speed: return 16;
        case FIELD_obj_heading: return 16;
        default: return 0;
    }
}

void SdsmPayloadDescriptor::setFieldArraySize(omnetpp::any_ptr object, int field, int size) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount()){
            base->setFieldArraySize(object, field, size);
            return;
        }
        field -= base->getFieldCount();
    }
    SdsmPayload *pp = omnetpp::fromAnyPtr<SdsmPayload>(object); (void)pp;
    switch (field) {
        default: throw omnetpp::cRuntimeError("Cannot set array size of field %d of class 'SdsmPayload'", field);
    }
}

const char *SdsmPayloadDescriptor::getFieldDynamicTypeString(omnetpp::any_ptr object, int field, int i) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldDynamicTypeString(object,field,i);
        field -= base->getFieldCount();
    }
    SdsmPayload *pp = omnetpp::fromAnyPtr<SdsmPayload>(object); (void)pp;
    switch (field) {
        default: return nullptr;
    }
}

std::string SdsmPayloadDescriptor::getFieldValueAsString(omnetpp::any_ptr object, int field, int i) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldValueAsString(object,field,i);
        field -= base->getFieldCount();
    }
    SdsmPayload *pp = omnetpp::fromAnyPtr<SdsmPayload>(object); (void)pp;
    switch (field) {
        case FIELD_payload: return oppstring2string(pp->getPayload());
        case FIELD_senderNodeIndex: return long2string(pp->getSenderNodeIndex());
        case FIELD_senderX: return double2string(pp->getSenderX());
        case FIELD_senderY: return double2string(pp->getSenderY());
        case FIELD_senderSpeed: return double2string(pp->getSenderSpeed());
        case FIELD_senderHeading: return double2string(pp->getSenderHeading());
        case FIELD_sendTimestamp: return double2string(pp->getSendTimestamp());
        case FIELD_message_id: return long2string(pp->getMessage_id());
        case FIELD_msg_cnt: return long2string(pp->getMsg_cnt());
        case FIELD_source_id: return long2string(pp->getSource_id(i));
        case FIELD_equipment_type: return long2string(pp->getEquipment_type());
        case FIELD_ref_pos_x: return double2string(pp->getRef_pos_x());
        case FIELD_ref_pos_y: return double2string(pp->getRef_pos_y());
        case FIELD_ref_pos_z: return double2string(pp->getRef_pos_z());
        case FIELD_sdsm_day: return long2string(pp->getSdsm_day());
        case FIELD_sdsm_time_of_day_ms: return long2string(pp->getSdsm_time_of_day_ms());
        case FIELD_numObjects: return long2string(pp->getNumObjects());
        case FIELD_obj_type: return long2string(pp->getObj_type(i));
        case FIELD_object_id: return long2string(pp->getObject_id(i));
        case FIELD_offset_x: return long2string(pp->getOffset_x(i));
        case FIELD_offset_y: return long2string(pp->getOffset_y(i));
        case FIELD_offset_z: return long2string(pp->getOffset_z(i));
        case FIELD_obj_speed: return long2string(pp->getObj_speed(i));
        case FIELD_obj_heading: return long2string(pp->getObj_heading(i));
        default: return "";
    }
}

void SdsmPayloadDescriptor::setFieldValueAsString(omnetpp::any_ptr object, int field, int i, const char *value) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount()){
            base->setFieldValueAsString(object, field, i, value);
            return;
        }
        field -= base->getFieldCount();
    }
    SdsmPayload *pp = omnetpp::fromAnyPtr<SdsmPayload>(object); (void)pp;
    switch (field) {
        case FIELD_payload: pp->setPayload((value)); break;
        case FIELD_senderNodeIndex: pp->setSenderNodeIndex(string2long(value)); break;
        case FIELD_senderX: pp->setSenderX(string2double(value)); break;
        case FIELD_senderY: pp->setSenderY(string2double(value)); break;
        case FIELD_senderSpeed: pp->setSenderSpeed(string2double(value)); break;
        case FIELD_senderHeading: pp->setSenderHeading(string2double(value)); break;
        case FIELD_sendTimestamp: pp->setSendTimestamp(string2double(value)); break;
        case FIELD_message_id: pp->setMessage_id(string2long(value)); break;
        case FIELD_msg_cnt: pp->setMsg_cnt(string2long(value)); break;
        case FIELD_source_id: pp->setSource_id(i,string2long(value)); break;
        case FIELD_equipment_type: pp->setEquipment_type(string2long(value)); break;
        case FIELD_ref_pos_x: pp->setRef_pos_x(string2double(value)); break;
        case FIELD_ref_pos_y: pp->setRef_pos_y(string2double(value)); break;
        case FIELD_ref_pos_z: pp->setRef_pos_z(string2double(value)); break;
        case FIELD_sdsm_day: pp->setSdsm_day(string2long(value)); break;
        case FIELD_sdsm_time_of_day_ms: pp->setSdsm_time_of_day_ms(string2long(value)); break;
        case FIELD_numObjects: pp->setNumObjects(string2long(value)); break;
        case FIELD_obj_type: pp->setObj_type(i,string2long(value)); break;
        case FIELD_object_id: pp->setObject_id(i,string2long(value)); break;
        case FIELD_offset_x: pp->setOffset_x(i,string2long(value)); break;
        case FIELD_offset_y: pp->setOffset_y(i,string2long(value)); break;
        case FIELD_offset_z: pp->setOffset_z(i,string2long(value)); break;
        case FIELD_obj_speed: pp->setObj_speed(i,string2long(value)); break;
        case FIELD_obj_heading: pp->setObj_heading(i,string2long(value)); break;
        default: throw omnetpp::cRuntimeError("Cannot set field %d of class 'SdsmPayload'", field);
    }
}

omnetpp::cValue SdsmPayloadDescriptor::getFieldValue(omnetpp::any_ptr object, int field, int i) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldValue(object,field,i);
        field -= base->getFieldCount();
    }
    SdsmPayload *pp = omnetpp::fromAnyPtr<SdsmPayload>(object); (void)pp;
    switch (field) {
        case FIELD_payload: return pp->getPayload();
        case FIELD_senderNodeIndex: return pp->getSenderNodeIndex();
        case FIELD_senderX: return pp->getSenderX();
        case FIELD_senderY: return pp->getSenderY();
        case FIELD_senderSpeed: return pp->getSenderSpeed();
        case FIELD_senderHeading: return pp->getSenderHeading();
        case FIELD_sendTimestamp: return pp->getSendTimestamp();
        case FIELD_message_id: return (omnetpp::intval_t)(pp->getMessage_id());
        case FIELD_msg_cnt: return pp->getMsg_cnt();
        case FIELD_source_id: return pp->getSource_id(i);
        case FIELD_equipment_type: return pp->getEquipment_type();
        case FIELD_ref_pos_x: return pp->getRef_pos_x();
        case FIELD_ref_pos_y: return pp->getRef_pos_y();
        case FIELD_ref_pos_z: return pp->getRef_pos_z();
        case FIELD_sdsm_day: return pp->getSdsm_day();
        case FIELD_sdsm_time_of_day_ms: return (omnetpp::intval_t)(pp->getSdsm_time_of_day_ms());
        case FIELD_numObjects: return pp->getNumObjects();
        case FIELD_obj_type: return pp->getObj_type(i);
        case FIELD_object_id: return pp->getObject_id(i);
        case FIELD_offset_x: return pp->getOffset_x(i);
        case FIELD_offset_y: return pp->getOffset_y(i);
        case FIELD_offset_z: return pp->getOffset_z(i);
        case FIELD_obj_speed: return pp->getObj_speed(i);
        case FIELD_obj_heading: return pp->getObj_heading(i);
        default: throw omnetpp::cRuntimeError("Cannot return field %d of class 'SdsmPayload' as cValue -- field index out of range?", field);
    }
}

void SdsmPayloadDescriptor::setFieldValue(omnetpp::any_ptr object, int field, int i, const omnetpp::cValue& value) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount()){
            base->setFieldValue(object, field, i, value);
            return;
        }
        field -= base->getFieldCount();
    }
    SdsmPayload *pp = omnetpp::fromAnyPtr<SdsmPayload>(object); (void)pp;
    switch (field) {
        case FIELD_payload: pp->setPayload(value.stringValue()); break;
        case FIELD_senderNodeIndex: pp->setSenderNodeIndex(omnetpp::checked_int_cast<int>(value.intValue())); break;
        case FIELD_senderX: pp->setSenderX(value.doubleValue()); break;
        case FIELD_senderY: pp->setSenderY(value.doubleValue()); break;
        case FIELD_senderSpeed: pp->setSenderSpeed(value.doubleValue()); break;
        case FIELD_senderHeading: pp->setSenderHeading(value.doubleValue()); break;
        case FIELD_sendTimestamp: pp->setSendTimestamp(value.doubleValue()); break;
        case FIELD_message_id: pp->setMessage_id(omnetpp::checked_int_cast<long>(value.intValue())); break;
        case FIELD_msg_cnt: pp->setMsg_cnt(omnetpp::checked_int_cast<int>(value.intValue())); break;
        case FIELD_source_id: pp->setSource_id(i,omnetpp::checked_int_cast<int>(value.intValue())); break;
        case FIELD_equipment_type: pp->setEquipment_type(omnetpp::checked_int_cast<int>(value.intValue())); break;
        case FIELD_ref_pos_x: pp->setRef_pos_x(value.doubleValue()); break;
        case FIELD_ref_pos_y: pp->setRef_pos_y(value.doubleValue()); break;
        case FIELD_ref_pos_z: pp->setRef_pos_z(value.doubleValue()); break;
        case FIELD_sdsm_day: pp->setSdsm_day(omnetpp::checked_int_cast<int>(value.intValue())); break;
        case FIELD_sdsm_time_of_day_ms: pp->setSdsm_time_of_day_ms(omnetpp::checked_int_cast<long>(value.intValue())); break;
        case FIELD_numObjects: pp->setNumObjects(omnetpp::checked_int_cast<int>(value.intValue())); break;
        case FIELD_obj_type: pp->setObj_type(i,omnetpp::checked_int_cast<int>(value.intValue())); break;
        case FIELD_object_id: pp->setObject_id(i,omnetpp::checked_int_cast<int>(value.intValue())); break;
        case FIELD_offset_x: pp->setOffset_x(i,omnetpp::checked_int_cast<int>(value.intValue())); break;
        case FIELD_offset_y: pp->setOffset_y(i,omnetpp::checked_int_cast<int>(value.intValue())); break;
        case FIELD_offset_z: pp->setOffset_z(i,omnetpp::checked_int_cast<int>(value.intValue())); break;
        case FIELD_obj_speed: pp->setObj_speed(i,omnetpp::checked_int_cast<int>(value.intValue())); break;
        case FIELD_obj_heading: pp->setObj_heading(i,omnetpp::checked_int_cast<int>(value.intValue())); break;
        default: throw omnetpp::cRuntimeError("Cannot set field %d of class 'SdsmPayload'", field);
    }
}

const char *SdsmPayloadDescriptor::getFieldStructName(int field) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldStructName(field);
        field -= base->getFieldCount();
    }
    switch (field) {
        default: return nullptr;
    };
}

omnetpp::any_ptr SdsmPayloadDescriptor::getFieldStructValuePointer(omnetpp::any_ptr object, int field, int i) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount())
            return base->getFieldStructValuePointer(object, field, i);
        field -= base->getFieldCount();
    }
    SdsmPayload *pp = omnetpp::fromAnyPtr<SdsmPayload>(object); (void)pp;
    switch (field) {
        default: return omnetpp::any_ptr(nullptr);
    }
}

void SdsmPayloadDescriptor::setFieldStructValuePointer(omnetpp::any_ptr object, int field, int i, omnetpp::any_ptr ptr) const
{
    omnetpp::cClassDescriptor *base = getBaseClassDescriptor();
    if (base) {
        if (field < base->getFieldCount()){
            base->setFieldStructValuePointer(object, field, i, ptr);
            return;
        }
        field -= base->getFieldCount();
    }
    SdsmPayload *pp = omnetpp::fromAnyPtr<SdsmPayload>(object); (void)pp;
    switch (field) {
        default: throw omnetpp::cRuntimeError("Cannot set field %d of class 'SdsmPayload'", field);
    }
}

}  // namespace messages
}  // namespace veins_ros_v2v

namespace omnetpp {

}  // namespace omnetpp

