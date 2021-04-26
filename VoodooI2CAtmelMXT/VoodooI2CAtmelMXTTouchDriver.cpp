//
//  VoodooI2CAtmelMXTTouchDriver.cpp
//  VoodooI2CAtmelMXT
//
//  Created by CoolStar on 4/24/18.
//  Copyright © 2018 CoolStar. All rights reserved.
//

#include "VoodooI2CAtmelMXTTouchDriver.hpp"
#include "atmel_mxt.h"

#define super IOService
OSDefineMetaClassAndStructors(VoodooI2CAtmelMXTTouchDriver, IOService);

#define MXT_DEBUG_FLAG (1)

void VoodooI2CAtmelMXTTouchDriver::free() {
    IOLog("%s:: VoodooI2CAtmel resources have been deallocated\n", getName());
    super::free();
}

void VoodooI2CAtmelMXTTouchDriver::handle_input_threaded() {
    if (!ready_for_input) {
        read_in_progress = false;
        return;
    }
    command_gate->attemptAction(OSMemberFunctionCast(IOCommandGate::Action, this, &VoodooI2CAtmelMXTTouchDriver::parse_ATML_report));
    read_in_progress = false;
}

bool VoodooI2CAtmelMXTTouchDriver::init(OSDictionary *properties) {
    transducers = NULL;
    if (!super::init(properties)) {
        return false;
    }
    transducers = OSArray::withCapacity(MXT_MAX_FINGERS);
    if (!transducers) {
        return false;
    }
    DigitiserTransducerType type = kDigitiserTransducerFinger;
    for (int i = 0; i < MXT_MAX_FINGERS; i++) {
        VoodooI2CDigitiserTransducer* transducer = VoodooI2CDigitiserTransducer::transducer(type, NULL);
        transducers->setObject(transducer);
    }
    awake = true;
    ready_for_input = false;
    read_in_progress = false;
    return true;
}

bool VoodooI2CAtmelMXTTouchDriver::init_device() {
    int blksize;
    uint32_t crc;
    IOReturn retVal = kIOReturnSuccess;
    
    /* mxt_device initial. */
    {
        mxt_device.multitouch = 0;
        mxt_device.num_touchids = 0;
        mxt_device.max_reportid = 0;
        mxt_device.range_x = 0;
        mxt_device.range_y = 0;
        
        mxt_device.t5_address = 0;
        mxt_device.t5_msg_size = 0;
        mxt_device.t6_reportid = 0;
        mxt_device.t6_address = 0;
        mxt_device.t7_address = 0;
        mxt_device.t9_reportid_min = 0;
        mxt_device.t9_reportid_max = 0;
        mxt_device.t19_reportid = 0;
        mxt_device.t19_key_nums = 0;
        mxt_device.t19_key_map = NULL;
        mxt_device.t44_address = 0;
        mxt_device.t100_reportid_min = 0;
        mxt_device.t100_reportid_max = 0;
    };

    retVal = mxt_read_reg(0, (UInt8 *)&core.info, sizeof(core.info));
    if (retVal != kIOReturnSuccess){
        IOLog("%s::Unable to read info\n", getName());
        return false;
    }
    
    core.nobjs = core.info.num_objects;
    
    if (core.nobjs < 0 || core.nobjs > 1024) {
        IOLog("%s::init_device nobjs (%d) out of bounds\n", getName(), core.nobjs);
        return false;
    }
    
    blksize = sizeof(core.info) +
    core.nobjs * sizeof(mxt_object);
    totsize = blksize + sizeof(mxt_raw_crc);
    
    core.buf = (uint8_t *)IOMalloc(totsize);
    
    retVal = mxt_read_reg(0, core.buf, totsize);
    if (retVal != kIOReturnSuccess){
        IOLog("%s::Unable to read buffer\n", getName());
        return false;
    }
    
    crc = obp_convert_crc((mxt_raw_crc *)((uint8_t *)core.buf + blksize));
    
    if (obp_crc24(core.buf, blksize) != crc) {
        IOLog("%s::init_device: configuration space "
              "crc mismatch %08x/%08x\n", getName(),
              crc, obp_crc24(core.buf, blksize));
    }
    else {
        IOLog("%s::init_device: CRC Matched!\n", getName());
    }
    
    core.objs = (mxt_object *)((uint8_t *)core.buf +
                               sizeof(core.info));
    
    msgprocobj = mxt_findobject(&core, MXT_GEN_MESSAGEPROCESSOR);
    if (!msgprocobj){
        IOLog("%s::Unable to find message processor\n", getName());
        return false;
    }
    cmdprocobj = mxt_findobject(&core, MXT_GEN_COMMANDPROCESSOR);
    if (!cmdprocobj){
        IOLog("%s::Unable to find command processor\n", getName());
        return false;
    }
    
    // IOLog("%s::[Wellen's] core.nobjs: %d\n", getName(), core.nobjs);
    int reportid = 1;
    for (int i = 0; i < core.nobjs; i++) {
        mxt_object *obj = &core.objs[i];
        uint8_t min_id, max_id;
        
        if (obj->num_report_ids) {
            min_id = reportid;
            reportid += obj->num_report_ids *
            mxt_obj_instances(obj);
            max_id = reportid - 1;
        }
        else {
            min_id = 0;
            max_id = 0;
        }
        
        // IOLog("%s::[Wellen's] %d --> %d:0x%X\n", getName(), i, obj->type, obj->start_address);
        
        switch (obj->type) {
            case MXT_GEN_MESSAGE_T5:
                if (mxt_device.info.family == 0x80 && mxt_device.info.version < 0x20) {
                    /*
                     * On mXT224 firmware versions prior to V2.0
                     * read and discard unused CRC byte otherwise
                     * DMA reads are misaligned.
                     */
                    mxt_device.t5_msg_size = mxt_obj_size(obj);
                }
                else {
                    /* CRC not enabled, so skip last byte */
                    mxt_device.t5_msg_size = mxt_obj_size(obj) - 1;
                }
                mxt_device.t5_address = obj->start_address;
                break;
            case MXT_GEN_COMMAND_T6:
                mxt_device.t6_reportid = min_id;
                mxt_device.t6_address = obj->start_address;
                break;
            case MXT_GEN_POWER_T7:
                mxt_device.t7_address = obj->start_address;
                break;
            case MXT_TOUCH_MULTI_T9:
                mxt_device.multitouch = MXT_TOUCH_MULTI_T9;
                mxt_device.t9_reportid_min = min_id;
                mxt_device.t9_reportid_max = max_id;
                mxt_device.num_touchids = obj->num_report_ids * mxt_obj_instances(obj);
                break;
            case MXT_SPT_MESSAGECOUNT_T44:
                mxt_device.t44_address = obj->start_address;
                break;
            case MXT_SPT_GPIOPWM_T19:
                mxt_device.t19_reportid = min_id;
                break;
            case MXT_TOUCH_MULTITOUCHSCREEN_T100:
                mxt_device.multitouch = MXT_TOUCH_MULTITOUCHSCREEN_T100;
                mxt_device.t100_reportid_min = min_id;
                mxt_device.t100_reportid_max = max_id;
                
                /* first two report IDs reserved */
                mxt_device.num_touchids = obj->num_report_ids - 2;
                break;
        }
    }
    
    mxt_device.max_reportid = reportid;
    if (mxt_device.multitouch == MXT_TOUCH_MULTITOUCHSCREEN_T100) {
        mxt_device.t19_key_nums = sizeof(t100_buttons);
        mxt_device.t19_key_map = t100_buttons;
    } else {
        mxt_device.t19_key_nums = sizeof(t9_buttons);
        mxt_device.t19_key_map = t9_buttons;
    }
    
    IOLog("%s:: T6_reportid: %d\n",getName(), mxt_device.t6_reportid);
    IOLog("%s:: T9_reportid_min: %d T9_reportid_max: %d\n",getName(), mxt_device.t9_reportid_min, mxt_device.t9_reportid_max);
    IOLog("%s:: T100_reportid_min: %d T100_reportid_max: %d\n",getName(), mxt_device.t100_reportid_min, mxt_device.t100_reportid_max);
    IOLog("%s:: T19_reportid: %d\n",getName(), mxt_device.t19_reportid);
    
    if (mxt_device.multitouch == MXT_TOUCH_MULTI_T9)
        mxt_read_t9_resolution();
    else if (mxt_device.multitouch == MXT_TOUCH_MULTITOUCHSCREEN_T100)
        mxt_read_t100_config();
    
    atmel_reset_device();
    
    if (mxt_device.multitouch == MXT_TOUCH_MULTITOUCHSCREEN_T100){
        mxt_set_t7_power_cfg(MXT_POWER_CFG_RUN);
    } else {
        mxt_object *obj = mxt_findobject(&core, MXT_TOUCH_MULTI_T9);
        mxt_write_object_off(obj, MXT_T9_CTRL, 0x83);
    }
    
    if (mt_interface){
        mt_interface->physical_max_x = mxt_device.range_x * 10;
        mt_interface->physical_max_y = mxt_device.range_y * 10;
        mt_interface->logical_max_x = mxt_device.range_x;
        mt_interface->logical_max_y = mxt_device.range_y;
    }
    return true;
}

void VoodooI2CAtmelMXTTouchDriver::interrupt_occurred(OSObject* owner, IOInterruptEventSource* src, int intCount) {
    if (read_in_progress)
        return;
    if (!awake)
        return;
    read_in_progress = true;
    thread_t new_thread;
    kern_return_t ret = kernel_thread_start(OSMemberFunctionCast(thread_continue_t, this, &VoodooI2CAtmelMXTTouchDriver::handle_input_threaded), this, &new_thread);
    if (ret != KERN_SUCCESS) {
        read_in_progress = false;
        IOLog("%s::Thread error while attemping to get input report\n", getName());
    } else {
        thread_deallocate(new_thread);
    }
}

int VoodooI2CAtmelMXTTouchDriver::mxt_process_get_usable_tip_id() {
    int tip_id = 0;
    for (int i = 0; i < MXT_MAX_FINGERS; i ++) {
        if (!tip_ids[i]) {
            tip_id = i;
            tip_ids[i] = true;
            break;
        }
    }
    
    return tip_id;
}

void VoodooI2CAtmelMXTTouchDriver::mxt_process_message_init() {
    /*int tip_id = mxt_process_get_usable_tip_id();
    
    AbsoluteTime timestamp;
    clock_get_uptime(&timestamp);
    
    for (int i = 0; i <= tip_id; i ++) {
        VoodooI2CDigitiserTransducer* transducer = OSDynamicCast(VoodooI2CDigitiserTransducer,  transducers->getObject(tip_id));
        if (transducer != NULL) {
            transducer->tip_pressure.update(0, timestamp);
            transducer->tip_switch.update(0, timestamp);
            transducer->physical_button.update(0, timestamp);
            transducer->is_valid = false;
        }
    }*/
    
    /* memset(tip_ids, false, sizeof(tip_ids)); */
}

void VoodooI2CAtmelMXTTouchDriver::mxt_precess_message_report(uint8_t count, AbsoluteTime timestamp) {
    if (!mt_interface) {
        return;
    }
    
    VoodooI2CMultitouchEvent event;
    event.contact_count = count;
    event.transducers = transducers;
#if MXT_DEBUG_FLAG
    VoodooI2CDigitiserTransducer* transducer = OSDynamicCast(VoodooI2CDigitiserTransducer,  transducers->getObject(0));
    if (transducer != NULL) {
        IOLog("[Wellen's] Once interrupt report, num.fingers = %u X:%u Y:%u\n", count, transducer->coordinates.x.value(), transducer->coordinates.y.value());
    } else {
        IOLog("[Wellen's] Once interrupt report, num.fingers = %u\n", count);
    }
#endif
    // Send the event into the multitouch interface.
    mt_interface->handleInterruptReport(event, timestamp);
    
    // tip_ids recovery.
    for (int i = 0; i < count; i ++) {
        tip_ids[i] = 0;
    }
}

IOReturn VoodooI2CAtmelMXTTouchDriver::mxt_process_t9_message(mxt_message *message, AbsoluteTime timestamp) {
    uint8_t *message_raw = (uint8_t *) message;
    
    uint8_t flags = message_raw[1];
    int posx = (message_raw[2] << 4) | ((message_raw[4] >> 4) & 0xf);
    int posy = (message_raw[3] << 4) | ((message_raw[4] & 0xf));
    
    /* Handle 10/12 bit switching */
    if (mxt_device.range_x < 1024) {
        posx >>= 2;
    }
    if (mxt_device.range_y < 1024) {
        posy >>= 2;
    }
    
    VoodooI2CDigitiserTransducer* transducer = OSDynamicCast(VoodooI2CDigitiserTransducer,  transducers->getObject(message->touch_t9.reportid));
    transducer->type = kDigitiserTransducerFinger;
    
    transducer->is_valid = true;
    
    if (mt_interface) {
        transducer->logical_max_x = mt_interface->logical_max_x;
        transducer->logical_max_y = mt_interface->logical_max_y;
    }
    
    uint8_t tipswitch = (flags & (MXT_T9_DETECT | MXT_T9_PRESS)) != 0;
    
    transducer->id = message->touch_t9.reportid;
    transducer->secondary_id = message->touch_t9.reportid;
    transducer->coordinates.x.update(posx, timestamp);
    transducer->coordinates.y.update(posy, timestamp);
    transducer->tip_switch.update(tipswitch, timestamp);
    
    tip_ids[message->touch_t9.reportid] = tipswitch;
    
    return kIOReturnSuccess;
}

IOReturn VoodooI2CAtmelMXTTouchDriver::mxt_process_t100_message(mxt_message *message, AbsoluteTime timestamp) {
#if 0 /* This id was caused bug on ATML. */
    uint8_t id = message[0] - (T100_reportid_min + 2);  /* first two report IDs reserved */
    if (id < 0) {
        return kIOReturnInvalid;
    }
#endif
    
    int tip_id = mxt_process_get_usable_tip_id();
    VoodooI2CDigitiserTransducer* transducer = OSDynamicCast(VoodooI2CDigitiserTransducer,  transducers->getObject(tip_id));
    if (transducer == NULL) {
        return kIOReturnError;
    }
    
    mxt_message_touch_t100 *t100_message = &message->touch_t100;
    
    if (mt_interface) {
        transducer->logical_max_x = mt_interface->logical_max_x;
        transducer->logical_max_y = mt_interface->logical_max_y;
    }
    
    uint8_t type = 0;
    bool valid = ((t100_message->flags) & MXT_T100_DETECT);
    uint8_t tip_switch = 0;
    uint8_t tip_pressure = 0;

    if (valid) {
        uint8_t type = (((t100_message->flags) & MXT_T100_TYPE_MASK) >> 4);
        switch (type) {
            case MXT_T100_TYPE_FINGER:
            case MXT_T100_TYPE_GLOVE:
                tip_switch = 1;
                transducer->type = kDigitiserTransducerFinger;
                tip_pressure = t100_message->area;
                break;
            case MXT_T100_TYPE_PASSIVE_STYLUS:
                tip_switch = 1;
                transducer->type = kDigitiserTransducerStylus;
                tip_pressure = t100_message->area;
                break;
            case MXT_T100_TYPE_HOVERING_FINGER:
                /* Ingore hovering touch. */
            case MXT_T100_TYPE_LARGE_TOUCH:
                /* Ingore suppressed touch. */
                break;
            default:
                break;
        }
    } else {
    }
    
    if (!tip_pressure && type != MXT_T100_TYPE_HOVERING_FINGER) {
        tip_pressure = MXT_PRESSURE_DEFAULT;
    }
    
    transducer->id = tip_id;
    transducer->secondary_id = tip_id;
    transducer->is_valid = valid;
    if (valid) {
        if (tip_switch) {
            transducer->coordinates.x.update(t100_message->x, timestamp);
            transducer->coordinates.y.update(t100_message->y, timestamp);
        } else {
            transducer->coordinates.x.update(transducer->coordinates.x.last.value, timestamp);
            transducer->coordinates.y.update(transducer->coordinates.y.last.value, timestamp);
        }
        transducer->tip_switch.update(tip_switch, timestamp);
#if MXT_DEBUG_FLAG
        IOLog("[Wellen's] update id = %u, type = %u, rawx = %d, rawy = %d\n", tip_id, type, t100_message->x, t100_message->y);
        IOLog("[Wellen's] update switch = %u, pressure = %u\n", tip_switch, tip_pressure);
#endif
    } else {
        transducer->tip_switch.update(0, timestamp);
#if MXT_DEBUG_FLAG
        IOLog("[Wellen's] ingore id = %u, type = %u, rawx = %d, rawy = %d\n", tip_id, type, t100_message->x, t100_message->y);
        IOLog("[Wellen's] ingore switch = %u, pressure = %u\n", tip_switch, tip_pressure);
        
        uint8_t *message_raw = (uint8_t *) message;
        IOLog("[Wellen's] Dump invalid:[%02X %02X %02X %02X %02X %02X %02X %02X\n]",
              message_raw[0], message_raw[1], message_raw[2], message_raw[3],
              message_raw[4], message_raw[5], message_raw[6], message_raw[7]);
#endif
    }
    
    return kIOReturnSuccess;
}

IOReturn VoodooI2CAtmelMXTTouchDriver::mxt_process_t19_message(mxt_message *message, AbsoluteTime timestamp) {
    int tip_id = mxt_process_get_usable_tip_id();
    VoodooI2CDigitiserTransducer* transducer = OSDynamicCast(VoodooI2CDigitiserTransducer,  transducers->getObject(tip_id));
    if (transducer == NULL) {
        return kIOReturnError;
    }
    
    if (mt_interface) {
        transducer->logical_max_x = mt_interface->logical_max_x;
        transducer->logical_max_y = mt_interface->logical_max_y;
    }
    
    int key_id = -1;
    bool button_click = false;
    for (int i = 0; i < mxt_device.t19_key_nums; i ++) {
        if (mxt_device.t19_key_map[i]) {
            button_click = !(message->any.data[0] & (1UL << i));
            if (button_click) {
                key_id = i;
                break;
            }
        }
    }
    
    transducer->id = tip_id;
    transducer->secondary_id = tip_id;
    transducer->is_valid = true;
    // transducer->coordinates.x.update(transducer->coordinates.x.last.value, timestamp);
    // transducer->coordinates.y.update(transducer->coordinates.y.last.value, timestamp);
    transducer->physical_button.update(button_click, timestamp);
#if MXT_DEBUG_FLAG
    IOLog("[Wellen's] t19 id = %u, click = %d key = %d\n", tip_id, button_click, key_id);
#endif
    
    return kIOReturnSuccess;
}

IOReturn VoodooI2CAtmelMXTTouchDriver::mxt_process_message_until_invalid(AbsoluteTime timestamp) {
    int count, read;
    uint8_t tries = 2;
    
    count = mxt_device.max_reportid;
    do {
        read = mxt_read_and_process_messages(count, timestamp);
        if (read < count) {
            return kIOReturnSuccess;
        }
    } while (--tries);
    return kIOReturnIOError;
}

IOReturn VoodooI2CAtmelMXTTouchDriver::mxt_process_message(mxt_message *message, AbsoluteTime timestamp) {
    if (!transducers) {
        return kIOReturnError;
    }
    
    uint8_t report_id = message->any.reportid;
    if (report_id == MXT_MESSAGE_INVALID) {
        return kIOReturnInvalid;
    }
    
    IOReturn io_ret = kIOReturnInvalid;
    
    if (report_id == mxt_device.t6_reportid) {
    } else if (report_id >= mxt_device.t9_reportid_min && report_id <= mxt_device.t9_reportid_max) {
        io_ret = mxt_process_t9_message(message, timestamp);
    } else if (report_id >= mxt_device.t100_reportid_min && report_id <= mxt_device.t100_reportid_max) {
        io_ret = mxt_process_t100_message(message, timestamp);
    } else if (report_id == mxt_device.t19_reportid) {
        io_ret = mxt_process_t19_message(message, timestamp);
    } else {}
    
    return io_ret;
}

int VoodooI2CAtmelMXTTouchDriver::mxt_read_and_process_messages(uint8_t count, AbsoluteTime timestamp) {
    uint8_t num_valid = 0;
    int i;
    if (count > mxt_device.max_reportid) {
        return -1;
    }

    int msg_buf_size = mxt_device.max_reportid * mxt_device.t5_msg_size;
    uint8_t *msg_buf = (uint8_t *) IOMalloc(msg_buf_size);
    
    for (int i = 0; i < msg_buf_size; i ++) {
        msg_buf[i] = MXT_MESSAGE_INVALID;
    }
    
    IOReturn io_ret = mxt_read_reg(mxt_device.t5_address, msg_buf, mxt_device.t5_msg_size * count);
    
    if (io_ret != kIOReturnSuccess) {
        IOLog("%s::Failed to read %u messages (%d)\n", getName(), count, io_ret);
        return -1;
    }
    
#if MXT_DEBUG_FLAG
    int dump_buf_size = msg_buf_size * 3 + 1;
    char *dump_buf = (char *) IOMalloc(dump_buf_size);
    // dump
    memset(dump_buf, 0x00, dump_buf_size);
    for (int i = 0; i < mxt_device.t5_msg_size * count; i ++) {
        snprintf(dump_buf + (3 * i), dump_buf_size - 1, "%02X ", msg_buf[i]);
    }
    IOLog("[Wellen's] Dump read: [%s]\n", dump_buf);
    IOFree(dump_buf, msg_buf_size * 3 + 1);
#endif
    
    for (i = 0; i < count; i ++) {
        mxt_message *message = (mxt_message *) (msg_buf + mxt_device.t5_msg_size * i);
        if (kIOReturnSuccess == mxt_process_message(message, timestamp)) {
            num_valid ++;
        }
    }
    
    IOFree(msg_buf, msg_buf_size);
    
    /* return number of messages read */
    return num_valid;
}

IOReturn VoodooI2CAtmelMXTTouchDriver::mxt_device_read_t44() {
    // IOLog("%s::device read t44\n", getName());
    uint8_t count, num_left;
    IOReturn io_ret = kIOReturnSuccess;
    
    int msg_buf_size = mxt_device.t5_msg_size + 1;
    uint8_t *msg_buf = (uint8_t *) IOMalloc(msg_buf_size);
    mxt_message *message = (mxt_message *) (&msg_buf[1]);
    
    mxt_process_message_init();
    
    /* Read T44 and T5 together */
    io_ret = mxt_read_reg(mxt_device.t44_address, msg_buf, mxt_device.t5_msg_size);

#if MXT_DEBUG_FLAG
    int dump_buf_size = msg_buf_size * 3 + 1;
    char *dump_buf = (char *) IOMalloc(dump_buf_size);
    // dump
    memset(dump_buf, 0x00, dump_buf_size);
    for (int i = 0; i < mxt_device.t5_msg_size; i ++) {
        snprintf(dump_buf + (3 * i), dump_buf_size - 1, "%02X ", msg_buf[i]);
    }
    IOLog("[Wellen's] Dump read: [%s]\n", dump_buf);
    IOFree(dump_buf, msg_buf_size * 3 + 1);
#endif
    
    if (io_ret != kIOReturnSuccess){
        goto End;
    }
    
    count = msg_buf[0];
    
    if (count == 0) {
        goto End;
    }
    
    if (count > mxt_device.max_reportid) {
        count = mxt_device.max_reportid;
    }
    
    AbsoluteTime timestamp;
    clock_get_uptime(&timestamp);
    
    io_ret = mxt_process_message(message, timestamp);
    if (io_ret != kIOReturnSuccess) {
        goto End;
    }
    
    num_left = count - 1;
    
    if (num_left) {
        num_left = mxt_read_and_process_messages(num_left, timestamp);
        if (num_left < 0){
            io_ret = kIOReturnIOError;
            goto End;
        }
    }
    
    mxt_precess_message_report(num_left + 1, timestamp);
    
End:
    IOFree(msg_buf, msg_buf_size);
    return io_ret;
}

IOReturn VoodooI2CAtmelMXTTouchDriver::mxt_device_read() {
    // IOLog("%s::device read\n", getName());
    int total_handled, num_handled;
    uint8_t count = last_message_count;
    
    if (count < 1 || count > mxt_device.max_reportid) {
        count = 1;
    }
    
    AbsoluteTime timestamp;
    clock_get_uptime(&timestamp);
    
    mxt_process_message_init();
    
    /* include final invalid message */
    total_handled = mxt_read_and_process_messages(count + 1, 0);
    if (total_handled < 0) {
        return kIOReturnIOError;
    } else if (total_handled <= count) {
        goto UpdateCount;
    }
    
    /* keep reading two msgs until one is invalid or reportid limit */
    do {
        num_handled = mxt_read_and_process_messages(2, timestamp);
        if (num_handled < 0) {
            return kIOReturnIOError;
        }
        total_handled += num_handled;
        if (num_handled < 2) {
            break;
        }
    } while (total_handled < mxt_device.num_touchids);

UpdateCount:
    last_message_count = total_handled;
    
    if (last_message_count > 0) {
        mxt_precess_message_report(last_message_count, timestamp);
    }
    
    return kIOReturnSuccess;
}

IOReturn VoodooI2CAtmelMXTTouchDriver::parse_ATML_report(){
    if (mxt_device.t44_address) {
        return mxt_device_read_t44();
    } else {
        return mxt_device_read();
    }
}

VoodooI2CAtmelMXTTouchDriver* VoodooI2CAtmelMXTTouchDriver::probe(IOService* provider, SInt32* score) {
    IOLog("%s::Touch probe\n", getName());
    if (!super::probe(provider, score)) {
        return NULL;
    }
    acpi_device = OSDynamicCast(IOACPIPlatformDevice, provider->getProperty("acpi-device"));
    if (!acpi_device) {
        IOLog("%s::Could not get ACPI device\n", getName());
        return NULL;
    }
    api = OSDynamicCast(VoodooI2CDeviceNub, provider);
    if (!api) {
        IOLog("%s::Could not get VoodooI2C API instance\n", getName());
        return NULL;
    }
    return this;
}

bool VoodooI2CAtmelMXTTouchDriver::publish_multitouch_interface() {
    mt_interface = new VoodooI2CMultitouchInterface();
    if (!mt_interface) {
        IOLog("%s::No memory to allocate VoodooI2CMultitouchInterface instance\n", getName());
        goto multitouch_exit;
    }
    if (!mt_interface->init(NULL)) {
        IOLog("%s::Failed to init multitouch interface\n", getName());
        goto multitouch_exit;
    }
    if (!mt_interface->attach(this)) {
        IOLog("%s::Failed to attach multitouch interface\n", getName());
        goto multitouch_exit;
    }
    if (!mt_interface->start(this)) {
        IOLog("%s::Failed to start multitouch interface\n", getName());
        goto multitouch_exit;
    }
    // Assume we are a touchscreen for now
    mt_interface->setProperty(kIOHIDDisplayIntegratedKey, true);
    // 0x03EB is Atmel's Vendor Id
    mt_interface->setProperty(kIOHIDVendorIDKey, 0x03EB, 32);
    mt_interface->setProperty(kIOHIDProductIDKey, 0x8A03, 32);
    return true;
multitouch_exit:
    unpublish_multitouch_interface();
    return false;
}

void VoodooI2CAtmelMXTTouchDriver::release_resources() {
    if (command_gate) {
        workLoop->removeEventSource(command_gate);
        command_gate->release();
        command_gate = NULL;
    }
    if (interrupt_source) {
        interrupt_source->disable();
        workLoop->removeEventSource(interrupt_source);
        interrupt_source->release();
        interrupt_source = NULL;
    }
    if (workLoop) {
        workLoop->release();
        workLoop = NULL;
    }
    if (acpi_device) {
        acpi_device->release();
        acpi_device = NULL;
    }
    if (api) {
        if (api->isOpen(this)) {
            api->close(this);
        }
        api->release();
        api = NULL;
    }
    if (transducers) {
        for (int i = 0; i < transducers->getCount(); i++) {
            OSObject* object = transducers->getObject(i);
            if (object) {
                object->release();
            }
        }
        OSSafeReleaseNULL(transducers);
    }
    if (totsize > 0)
        IOFree(core.buf, totsize);
}

IOReturn VoodooI2CAtmelMXTTouchDriver::setPowerState(unsigned long longpowerStateOrdinal, IOService* whatDevice) {
    if (whatDevice != this)
        return kIOReturnInvalid;
    if (longpowerStateOrdinal == 0) {
        if (awake) {
            awake = false;
            for (;;) {
                if (!read_in_progress) {
                    break;
                }
                IOSleep(10);
            }
            IOLog("%s::Going to sleep\n", getName());
        }
    } else {
        if (!awake) {
            atmel_reset_device();
            awake = true;
            IOLog("%s::Woke up and reset device\n", getName());
        }
    }
    return kIOPMAckImplied;
}

bool VoodooI2CAtmelMXTTouchDriver::start(IOService* provider) {
    if (!super::start(provider)) {
        return false;
    }
    workLoop = this->getWorkLoop();
    if (!workLoop) {
        IOLog("%s::Could not get a IOWorkLoop instance\n", getName());
        return false;
    }
    workLoop->retain();
    command_gate = IOCommandGate::commandGate(this);
    if (!command_gate || (workLoop->addEventSource(command_gate) != kIOReturnSuccess)) {
        IOLog("%s::Could not open command gate\n", getName());
        goto start_exit;
    }
    acpi_device->retain();
    api->retain();
    if (!api->open(this)) {
        IOLog("%s::Could not open API\n", getName());
        goto start_exit;
    }
    // set interrupts AFTER device is initialised
    interrupt_source = IOInterruptEventSource::interruptEventSource(this, OSMemberFunctionCast(IOInterruptEventAction, this, &VoodooI2CAtmelMXTTouchDriver::interrupt_occurred), api, 0);
    if (!interrupt_source) {
        IOLog("%s::Could not get interrupt event source\n", getName());
        goto start_exit;
    }
    publish_multitouch_interface();
    if (!init_device()) {
        IOLog("%s::Failed to init device\n", getName());
        return NULL;
    }
    workLoop->addEventSource(interrupt_source);
    interrupt_source->enable();
    PMinit();
    api->joinPMtree(this);
    registerPowerDriver(this, VoodooI2CIOPMPowerStates, kVoodooI2CIOPMNumberPowerStates);
    IOSleep(100);
    ready_for_input = true;
    setProperty("VoodooI2CServices Supported", OSBoolean::withBoolean(true));
    IOLog("%s::VoodooI2CAtmelMXT has started\n", getName());
    mt_interface->registerService();
    registerService();
    return true;
start_exit:
    release_resources();
    return false;
}

void VoodooI2CAtmelMXTTouchDriver::stop(IOService* provider) {
    release_resources();
    unpublish_multitouch_interface();
    PMstop();
    IOLog("%s::VoodooI2CAtmelMXT has stopped\n", getName());
    super::stop(provider);
}

void VoodooI2CAtmelMXTTouchDriver::unpublish_multitouch_interface() {
    if (mt_interface) {
        mt_interface->stop(this);
        mt_interface->release();
        mt_interface = NULL;
    }
}

size_t VoodooI2CAtmelMXTTouchDriver::mxt_obj_size(const mxt_object *obj)
{
    return obj->size_minus_one + 1;
}

size_t VoodooI2CAtmelMXTTouchDriver::mxt_obj_instances(const mxt_object *obj)
{
    return obj->instances_minus_one + 1;
}

mxt_object *VoodooI2CAtmelMXTTouchDriver::mxt_findobject(struct mxt_rollup *core, int type)
{
    int i;
    
    for (i = 0; i < core->nobjs; ++i) {
        if (core->objs[i].type == type)
            return(&core->objs[i]);
    }
    return NULL;
}

IOReturn VoodooI2CAtmelMXTTouchDriver::mxt_read_reg(UInt16 reg, UInt8 *rbuf, int len)
{
    UInt8 wreg[2];
    wreg[0] = reg & 255;
    wreg[1] = reg >> 8;
    
    IOReturn retVal = kIOReturnSuccess;
    retVal = api->writeReadI2C(reinterpret_cast<UInt8*>(wreg), sizeof(wreg), rbuf, len);
    return retVal;
}

IOReturn VoodooI2CAtmelMXTTouchDriver::mxt_write_reg_buf(UInt16 reg, UInt8 *xbuf, int len)
{
    UInt8 wreg[2];
    wreg[0] = reg & 255;
    wreg[1] = reg >> 8;
    
    UInt8 *intermbuf = (uint8_t *)IOMalloc(sizeof(wreg) + len);
    memcpy(intermbuf, wreg, sizeof(wreg));
    memcpy(intermbuf + sizeof(wreg), xbuf, len);
    
    IOReturn retVal = api->writeI2C(intermbuf, sizeof(wreg) + len);
    IOFree(intermbuf, sizeof(wreg) + len);
    return retVal;
}

IOReturn VoodooI2CAtmelMXTTouchDriver::mxt_write_reg(UInt16 reg, UInt8 val)
{
    return mxt_write_reg_buf(reg, &val, 1);
}

IOReturn
VoodooI2CAtmelMXTTouchDriver::mxt_write_object_off(mxt_object *obj,
                                                    int offset, UInt8 val)
{
    uint16_t reg = obj->start_address;
    
    reg += offset;
    return mxt_write_reg(reg, val);
}

void VoodooI2CAtmelMXTTouchDriver::atmel_reset_device()
{
    for (int i = 0; i < MXT_MAX_FINGERS; i++){
        this->tip_ids[i] = false;
    }

    mxt_write_object_off(cmdprocobj, MXT_CMDPROC_RESET_OFF, 1);
    IOLog("%s::Reset sent!\n", getName());
    IOSleep(100);
}

IOReturn VoodooI2CAtmelMXTTouchDriver::mxt_set_t7_power_cfg(UInt8 sleep)
{
    t7_config *new_config;
    t7_config deepsleep;
    deepsleep.active = deepsleep.idle = 0;
    t7_config active;
    active.active = 20;
    active.idle = 100;
    
    if (sleep == MXT_POWER_CFG_DEEPSLEEP)
        new_config = &deepsleep;
    else
        new_config = &active;
    return mxt_write_reg_buf(mxt_device.t7_address, (UInt8 *)new_config, sizeof(t7_cfg));
}

IOReturn VoodooI2CAtmelMXTTouchDriver::mxt_read_t9_resolution()
{
    IOReturn retVal = kIOReturnSuccess;
    
    t9_range range;
    unsigned char orient;
    
    mxt_object *resolutionobject = mxt_findobject(&core, MXT_TOUCH_MULTI_T9);
    
    if (!resolutionobject){
        IOLog("%s::Unable to find T9 object\n", getName());
        return kIOReturnInvalid;
    }
    
    retVal = mxt_read_reg(resolutionobject->start_address + MXT_T9_RANGE, (UInt8 *)&range, sizeof(range));
    if (retVal != kIOReturnSuccess){
        IOLog("%s::Unable to read T9 range\n", getName());
        return retVal;
    }
    
    retVal = mxt_read_reg(resolutionobject->start_address + MXT_T9_ORIENT, &orient, 1);
    if (retVal != kIOReturnSuccess){
        IOLog("%s::Unable to read T9 orientation\n", getName());
        return retVal;
    }
    
    /* Handle default values */
    if (range.x == 0)
        range.x = 1024;
    
    if (range.y == 0)
        range.y = 1024;
    
    if (orient & MXT_T9_ORIENT_SWITCH) {
        mxt_device.range_x = range.y;
        mxt_device.range_y = range.x;
    }
    else {
        mxt_device.range_x = range.x;
        mxt_device.range_y = range.y;
    }
    IOLog("%s:: Screen Size: X: %d Y: %d\n", getName(), mxt_device.range_x, mxt_device.range_y);
    return retVal;
}

IOReturn VoodooI2CAtmelMXTTouchDriver::mxt_read_t100_config()
{
    IOReturn retVal = kIOReturnSuccess;
    uint16_t range_x, range_y;
    uint8_t cfg, tchaux;
    uint8_t aux;
    
    mxt_object *resolutionobject = mxt_findobject(&core, MXT_TOUCH_MULTITOUCHSCREEN_T100);
    if (!resolutionobject){
        IOLog("%s::Unable to find T100 object\n", getName());
        return kIOReturnInvalid;
    }
    
    /* read touchscreen dimensions */
    retVal = mxt_read_reg(resolutionobject->start_address + MXT_T100_XRANGE, (UInt8 *)&range_x, sizeof(range_x));
    if (retVal != kIOReturnSuccess){
        IOLog("%s::Unable to read T100 xrange\n", getName());
        return retVal;
    }
    
    retVal = mxt_read_reg(resolutionobject->start_address + MXT_T100_YRANGE, (UInt8 *)&range_y, sizeof(range_y));
    if (retVal != kIOReturnSuccess){
        IOLog("%s::Unable to read T100 yrange\n", getName());
        return retVal;
    }
    
    /* read orientation config */
    retVal = mxt_read_reg(resolutionobject->start_address + MXT_T100_CFG1, &cfg, 1);
    if (retVal != kIOReturnSuccess){
        IOLog("%s::Unable to read T100 orientation config\n", getName());
        return retVal;
    }
    
    if (cfg & MXT_T100_CFG_SWITCHXY) {
        mxt_device.range_x = range_y;
        mxt_device.range_y = range_x;
    }
    else {
        mxt_device.range_x = range_x;
        mxt_device.range_y = range_y;
    }
    
    retVal = mxt_read_reg(resolutionobject->start_address + MXT_T100_TCHAUX, &tchaux, 1);
    if (retVal != kIOReturnSuccess){
        IOLog("%s::Unable to read T100 aux bits\n", getName());
        return retVal;
    }
    
    aux = 6;
    
    if (tchaux & MXT_T100_TCHAUX_VECT)
        t100_aux_vect = aux++;
    
    if (tchaux & MXT_T100_TCHAUX_AMPL)
        t100_aux_ampl = aux++;
    
    if (tchaux & MXT_T100_TCHAUX_AREA)
        t100_aux_area = aux++;
    IOLog("%s::Screen Size T100: Range X: %u Y: %u\n", getName(), mxt_device.range_x, mxt_device.range_y);
    return retVal;
}
